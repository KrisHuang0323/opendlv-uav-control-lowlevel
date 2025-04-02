/*
* Copyright (C) 2018  Christian Berger
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <cmath>
#include <random>
#include <chrono>

std::mutex coutMutex;
void print(const std::string& message) {
    std::lock_guard<std::mutex> lock(coutMutex);
    std::cout << message << std::endl;
}
 
void Takeoff(cluon::OD4Session &od4, float height, int duration){
    std::ostringstream oss;
    oss << "Taking off to height: " << height;
    print( oss.str() );       
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Goto(cluon::OD4Session &od4, float x, float y, float z, float yaw, int duration, int relative = 1, bool isdelay = false, bool verbose = true){
    if ( verbose ){
        std::ostringstream oss;
        oss << "Go to position : x: "<< x << " ,y: " << y << " ,z: " << z << " , yaw: " << yaw;
        print( oss.str() );       
    }
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.x(x);
    cfcommand.y(y);
    cfcommand.z(z);
    cfcommand.yaw(yaw);
    cfcommand.time(duration);
    // cfcommand.relative(relative);
    od4.send(cfcommand, sampleTime, 3);
    if ( isdelay ){
        std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
    }
}

void Landing(cluon::OD4Session &od4, float height, int duration){
    std::ostringstream oss;
    oss << "Landing to height: " << height;
    print( oss.str() );      
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Stopping(cluon::OD4Session &od4){
    std::ostringstream oss;
    oss << "Stopping." << height;
    print( oss.str() );      
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    od4.send(cfcommand, sampleTime, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

float angleDifference(float angle1, float angle2) {
    float diff = angle2 - angle1;
    diff = std::fmod(diff + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    diff -= M_PI;
    return diff;
}

float wrap_angle(float angle) {
    return (angle > M_PI) ? (angle - 2 * M_PI) : angle;
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << "You should include the cid to start communicate in OD4Session" << std::endl;
        return retCode;
    }

    const float homing_batterythreshold = (commandlineArguments.count("hbat") != 0) ? std::stof(commandlineArguments["hbat"]) : 3.35;
    const float takeoff_batterythreshold = (commandlineArguments.count("tbat") != 0) ? std::stof(commandlineArguments["tbat"]) : 3.6;

    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    // cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    auto od4 = std::make_shared<cluon::OD4Session>(static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])));

    std::atomic<bool> staticAvoidanceActive(false);
    std::atomic<bool> dynamicAvoidanceActive(false);
    std::atomic<bool> frontReachingActive(false);
    std::atomic<bool> targetFindingActive(false);
    std::atomic<bool> lookingAroundActive(false);
    
    // Handler to receive distance readings (realized as C++ lambda).
    std::mutex distancesMutex;
    float front{0};
    float rear{0};
    float left{0};
    float right{0};
    auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
        // Store distance readings.
        std::lock_guard<std::mutex> lck(distancesMutex);
        switch (senderStamp) {
            case 0: front = dr.distance(); break;
            case 1: rear = dr.distance(); break;
            case 2: left = dr.distance(); break;
            case 3: right = dr.distance(); break;
        }
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

    // Handler to receive state readings (realized as C++ lambda).
    struct State {
        float yaw;
        float battery_state;
    };
    std::mutex stateMutex;
    State cur_state{0.0f, 0.0f};
    auto onStateRead = [&stateMutex, &cur_state](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::sensation::CrazyFlieState cfState = cluon::extractMessage<opendlv::logic::sensation::CrazyFlieState>(std::move(env));
        // Store distance readings.
        std::lock_guard<std::mutex> lck(stateMutex);
        cur_state.yaw = cfState.cur_yaw();
        cur_state.battery_state = cfState.battery_state();
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::sensation::CrazyFlieState::ID(), onStateRead);

    float dist_target{-1.0f};
    float dist_obs{-1.0f};
    float dist_chpad{-1.0f};
    std::mutex distMutex;
    auto onDistRead = [&distMutex, &dist_target, &dist_obs, &dist_chpad](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::action::PreviewPoint pPtmessage = cluon::extractMessage<opendlv::logic::action::PreviewPoint>(std::move(env));
        
        // Store distance readings.
        std::lock_guard<std::mutex> lck(distMutex);
        if ( senderStamp == 0 ){
            dist_target = pPtmessage.distance();
        }
        else if ( senderStamp == 1 ){
            dist_obs = pPtmessage.distance();
        }
        else if ( senderStamp == 2 ){
            dist_chpad = pPtmessage.distance();
        }
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::action::PreviewPoint::ID(), onDistRead);

    float aimDirection_target{-4.0f};
    float aimDirection_obs{-4.0f};
    float aimDirection_chpad{-4.0f};
    std::mutex aimDirectionMutex;
    auto onAimDirectionRead = [&aimDirectionMutex, &aimDirection_target, &aimDirection_obs, &aimDirection_chpad](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::action::AimDirection aDirmessage = cluon::extractMessage<opendlv::logic::action::AimDirection>(std::move(env));
        
        // Store aim direction readings.
        std::lock_guard<std::mutex> lck(aimDirectionMutex);
        if ( senderStamp == 0 ){
            aimDirection_target = aDirmessage.azimuthAngle();
        }
        else if ( senderStamp == 1 ){
            aimDirection_obs = aDirmessage.azimuthAngle();
        }
        else if ( senderStamp == 2 ){
            aimDirection_chpad = aDirmessage.azimuthAngle();
        }
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::action::AimDirection::ID(), onAimDirectionRead);

    // Takeoff flags
    bool hasTakeoff = false;
    // float takeoff_batterythreshold = 3.6f;

    // Varibles to record current valid ranges
    struct distPathState {
        float dist_valid_onPath;
        float dist_valid_devPath;
    };
    std::vector<distPathState> distPathstate_vec;
    struct ValidWay { 
        float toLeft;
        float toRight;
        float toRear;
    };
    ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
    bool on_GoTO_MODE = false;
    bool on_TURNING_MODE = false;

    // Variables for static obstacles avoidance
    float safe_endreach_dist = 0.27;
    float safe_endreach_LR_dist = 0.1;
    float cur_distToMove{0.0f};
    int time_toMove = 1;
    struct preDist {
        float left;
        float right;
    };
    preDist cur_preDist = {-1.0f, -1.0f};
    struct ReachEndState {
        bool reachFront;
        bool reachLeft;
        bool reachRight;
        bool reachRear;
    };
    ReachEndState cur_reachEndState = { false, false, false, false };

    // Variables for dynamic obstacles avoidance
    float dodgeDist{0.0f};
    float dodgeDist_UP{0.0f};
    bool has_dodgeToRear = false;
    enum DodgeType {
        DODGE_STOP,
        DODGE_LEFT,
        DODGE_RIGHT,
        DODGE_REAR,
        DODGE_UP,
        DODGE_NONE
    };
    DodgeType cur_dodgeType = DODGE_NONE;
    struct obsState {
        float dist_obs;
        float aimDirection_obs;
    };
    obsState cur_obsState = { -1.0f, -1.0f };
    bool has_possibleInterrupt = false;
    bool has_InterruptNeedToReDo = false;

    // Variables for front reaching
    struct pathReachingState {
        bool pathReadyToGo;
        bool pathOnGoing;
        float startFront;
    };
    pathReachingState cur_pathReachingState = {false, false, -1.0f};

    // Variables for target finding
    struct targetCheckState {
        bool aimTurnStarted;
        bool pointToTarget;
        bool turnStarted;
        float startAngle;
        float cur_aimDiff;
        float ang_toTurn;
        float targetAngle;
    };
    targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f};
    float start_turning_angle{0.0f};

    // Variables for homing
    // float homing_batterythreshold = 3.35f;
    // float homing_batterythreshold = 2.5f;

    // Variables for looking around
    struct angleFrontState {
        float angle;
        float front;
    };    
    std::vector<angleFrontState> angleFrontState_vec;
    struct lookAroundState {
        bool clearPathCheckStarted;
        bool turnStarted;
        bool smallToBig;
        float preAngle;
        float startAngle;
        float targetAngle;
        int nTimer;
    };
    lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
    float ori_front{0.0f};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution (0, 1);

    // Timer to record time of each behaviour
    auto taskStartTime = std::chrono::high_resolution_clock::now();
    auto taskEndTime = std::chrono::high_resolution_clock::now();
    auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
    auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
    auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
    auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
    auto targetFindingStartTime = std::chrono::high_resolution_clock::now();
    auto targetFindingEndTime = std::chrono::high_resolution_clock::now();
    auto frontReachingStartTime = std::chrono::high_resolution_clock::now();
    auto frontReachingEndTime = std::chrono::high_resolution_clock::now();
    auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
    auto lookAroundEndTime = std::chrono::high_resolution_clock::now();


    // 初始動作：起飛
    if ( hasTakeoff == false ){
        if ( cur_state.battery_state > takeoff_batterythreshold ){
            Takeoff(od4, 1.0f, 3);
            hasTakeoff = true;
            taskStartTime = std::chrono::high_resolution_clock::now();
        }
        else{
            std::cout <<" Battery is too low for taking off..." << std::endl;
            continue;
        }
    }
    Takeoff(1.0f, 3);

    // 建立各行為的執行緒
    std::thread ValidDirectionCheckTask([&value]() {
        std::cout << "執行緒中的值: " << value << std::endl;
    });

    std::thread t_dynamic(dynamicObstacleAvoidance);
    std::thread t_static(staticObstacleAvoidance);
    std::thread t_target(targetFinding);
    std::thread t_front(frontReaching);
    std::thread t_look(lookAround);
    std::thread t_arb(arbitrator);

    // 模擬主迴圈：更新感測器數值（實際上會由回呼函式更新）
    for (int i = 0; i < 50; i++) {
        // 模擬前方距離隨機變化 (0.5 ~ 1.0 米)
        sensorFront = 0.5f + static_cast<float>(rand() % 100) / 200.0f;
        sensorLeft = 0.5f;
        sensorRight = 0.5f;
        sensorRear = 0.5f;
        // 每 10 次迴圈模擬一次目標出現
        if (i % 10 == 0) {
            targetDistance = 0.8f;
            targetAngle = 0.1f;
        } else {
            targetDistance = -1.0f;
            targetAngle = -4.0f;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 結束動作：降落與停止
    Landing(0.0f, 3);
    Stopping();

    // 由於各行為執行緒為無限迴圈，這裡用 detach 讓程式結束時自動回收 (實際應用中需適當管理執行緒退出)
    t_dynamic.detach();
    t_static.detach();
    t_target.detach();
    t_front.detach();
    t_look.detach();
    t_arb.detach();

    return 0;
}