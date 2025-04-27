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

std::mutex od4Mutex; // Mutex to protect shared access to od4
void Takeoff(std::shared_ptr<cluon::OD4Session> od4, float height, int duration){
    std::cout << "Taking off to height: " << height << std::endl;        
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    {
    std::lock_guard<std::mutex> lck(od4Mutex);
    od4->send(cfcommand, sampleTime, 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Goto(std::shared_ptr<cluon::OD4Session> od4, float x, float y, float z, float yaw, int duration, int relative = 1, bool isdelay = false, bool verbose = true){
    if ( verbose ){
        std::cout << "Go to position : x: "<< x << " ,y: " << y << " ,z: " << z << " , yaw: " << yaw << std::endl;
    }
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.x(x);
    cfcommand.y(y);
    cfcommand.z(z);
    cfcommand.yaw(yaw);
    cfcommand.time(duration);
    // cfcommand.relative(relative);
    {
    std::lock_guard<std::mutex> lck(od4Mutex);
    od4->send(cfcommand, sampleTime, 3);
    }
    if ( isdelay ){
        std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
    }
}

void Landing(std::shared_ptr<cluon::OD4Session> od4, float height, int duration){
    std::cout << "Landing to height: " << height << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    {
    std::lock_guard<std::mutex> lck(od4Mutex);
    od4->send(cfcommand, sampleTime, 1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Stopping(std::shared_ptr<cluon::OD4Session> od4){
    std::cout << "Stopping." << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    {
    std::lock_guard<std::mutex> lck(od4Mutex);
    od4->send(cfcommand, sampleTime, 2);
    }
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

     int16_t maptype{0};
     if ( (0 == commandlineArguments.count("maptype")) ) {
         std::cerr << "You should include the maptype to start..." << std::endl;
         return retCode;
     }
     else{
         maptype = static_cast<int16_t>(std::stoi(commandlineArguments["maptype"]));
     }

     const float landing_batterythreshold = (commandlineArguments.count("lbat") != 0) ? std::stof(commandlineArguments["lbat"]) : 3.0;
     const float homing_batterythreshold = (commandlineArguments.count("hbat") != 0) ? std::stof(commandlineArguments["hbat"]) : 3.35;
     const float takeoff_batterythreshold = (commandlineArguments.count("tbat") != 0) ? std::stof(commandlineArguments["tbat"]) : 3.6;
 
     // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
     std::shared_ptr<cluon::OD4Session> od4 = std::make_shared<cluon::OD4Session>(static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])));
    //  cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
 
     // Handler to receive distance readings (realized as C++ lambda).
    //  std::mutex distancesMutex;     
     struct sensorReadStruct{
        std::atomic<float> front{0};
        std::atomic<float> rear{0};
        std::atomic<float> left{0};
        std::atomic<float> right{0};
        std::atomic<float> cur_state_yaw{0.0f};
        std::atomic<float> cur_state_battery_state{0.0f};
        std::atomic<float> dist_target{-1.0f};
        std::atomic<float> dist_obs{-1.0f};
        std::atomic<float> dist_chpad{-1.0f};
        std::atomic<float> aimDirection_target{-4.0f};
        std::atomic<float> aimDirection_obs{-4.0f};
        std::atomic<float> aimDirection_chpad{-4.0f};
        std::atomic<float> closeBallTimer{0.0f};
        std::atomic<int16_t> closeBallCount{0}; 
        std::atomic<float> closeStaticObsTimer{0.0f};
        std::atomic<int16_t> closeStaticObsCount{0}; 
        std::atomic<int16_t> nTargetTimer{0};
        std::atomic<int16_t> is_chpad_found{0};  
     };
     sensorReadStruct cur_sensorReadStruct;
     auto onDistance = [&cur_sensorReadStruct](cluon::data::Envelope &&env){
         auto senderStamp = env.senderStamp();
         // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
         opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
         // Store distance readings.
        //  std::lock_guard<std::mutex> lck(distancesMutex);
         switch (senderStamp) {
             case 0: cur_sensorReadStruct.front = dr.distance(); break;
             case 1: cur_sensorReadStruct.rear = dr.distance(); break;
             case 2: cur_sensorReadStruct.left = dr.distance(); break;
             case 3: cur_sensorReadStruct.right = dr.distance(); break;
         }
     };
     // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
     od4->dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);
 
     // Handler to receive state readings (realized as C++ lambda).
    //  struct State {
    //     std::atomic<float> yaw{0.0f};
    //     std::atomic<float> battery_state{0.0f};
    //  };
    //  std::mutex stateMutex;
     auto onStateRead = [&cur_sensorReadStruct](cluon::data::Envelope &&env){
         auto senderStamp = env.senderStamp();
         // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
         opendlv::logic::sensation::CrazyFlieState cfState = cluon::extractMessage<opendlv::logic::sensation::CrazyFlieState>(std::move(env));
         // Store distance readings.
        //  std::lock_guard<std::mutex> lck(stateMutex);
        cur_sensorReadStruct.cur_state_yaw = cfState.cur_yaw();
        cur_sensorReadStruct.cur_state_battery_state = cfState.battery_state();
        //  std::cout <<" Current angle: " <<  cur_state_r.yaw << std::endl; 
     };
     // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
     od4->dataTrigger(opendlv::logic::sensation::CrazyFlieState::ID(), onStateRead);
 
    //  std::mutex distMutex;
     auto onDistRead = [&cur_sensorReadStruct](cluon::data::Envelope &&env){
         auto senderStamp = env.senderStamp();
         // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
         opendlv::logic::action::PreviewPoint pPtmessage = cluon::extractMessage<opendlv::logic::action::PreviewPoint>(std::move(env));
         
         // Store distance readings.
        //  std::lock_guard<std::mutex> lck(distMutex);
         if ( senderStamp == 0 ){
            cur_sensorReadStruct.dist_target = pPtmessage.distance();
         }
         else if ( senderStamp == 1 ){
            cur_sensorReadStruct.dist_obs = pPtmessage.distance();
         }
         else if ( senderStamp == 2 ){
            cur_sensorReadStruct.dist_chpad = pPtmessage.distance();
         }
     };
     // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
     od4->dataTrigger(opendlv::logic::action::PreviewPoint::ID(), onDistRead);
 
    //  std::mutex aimDirectionMutex;
     auto onAimDirectionRead = [&cur_sensorReadStruct](cluon::data::Envelope &&env){
         auto senderStamp = env.senderStamp();
         // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
         opendlv::logic::action::AimDirection aDirmessage = cluon::extractMessage<opendlv::logic::action::AimDirection>(std::move(env));
         
         // Store aim direction readings.
        //  std::lock_guard<std::mutex> lck(aimDirectionMutex);
         if ( senderStamp == 0 ){
            cur_sensorReadStruct.aimDirection_target = aDirmessage.azimuthAngle();
         }
         else if ( senderStamp == 1 ){
            cur_sensorReadStruct.aimDirection_obs = aDirmessage.azimuthAngle();
         }
         else if ( senderStamp == 2 ){
            cur_sensorReadStruct.aimDirection_chpad = aDirmessage.azimuthAngle();
         }
     };
     // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
     od4->dataTrigger(opendlv::logic::action::AimDirection::ID(), onAimDirectionRead);
 
     auto onRewardRecordRead = [&cur_sensorReadStruct](cluon::data::Envelope &&env){
         auto senderStamp = env.senderStamp();
         // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
         opendlv::logic::sensation::RewardRecord rRecordmessage = cluon::extractMessage<opendlv::logic::sensation::RewardRecord>(std::move(env));
         
         // Store aim direction readings.
        //  std::lock_guard<std::mutex> lck(aimDirectionMutex);
         if ( senderStamp == 0 ){
            cur_sensorReadStruct.closeBallTimer = rRecordmessage.too_close_ball_timer();
            cur_sensorReadStruct.closeBallCount = rRecordmessage.too_close_ball_count();
            cur_sensorReadStruct.closeStaticObsTimer = rRecordmessage.too_close_staticobs_timer();
            cur_sensorReadStruct.closeStaticObsCount = rRecordmessage.too_close_staticobs_count();
            cur_sensorReadStruct.nTargetTimer = rRecordmessage.target_found_count();
            cur_sensorReadStruct.is_chpad_found = rRecordmessage.is_chpad_found();
         }
     };
     // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
     od4->dataTrigger(opendlv::logic::sensation::RewardRecord::ID(), onRewardRecordRead);

    // Takeoff flags
    bool hasTakeoff = false;
    // float takeoff_batterythreshold = 3.6f;

    // Constant variables that won't change(for now, migt be tuned later)
    struct constVarStruct{
        const float safe_endreach_ultimate_dist = 0.08;
        const float safe_endreach_dist = 0.25;
        const float safe_endreach_LR_dist = 0.1;
        int16_t nTargetCount;
    };
    constVarStruct cur_constVarStruct;
    int nCount = 3;
    if ( maptype == 1 )
        nCount = 2;
    cur_constVarStruct.nTargetCount = nCount;

    // Variables for suppressing
    std::mutex suppressMutex;
    struct suppressStruct{
        std::atomic<bool> isObsStaticDominating{false};
        std::atomic<bool> isObsDynamicDominating{false};
        std::atomic<bool> isTargetFindingDominating{false};
        std::atomic<bool> isFrontReachingDominating{false};
        std::atomic<bool> isSupressFrontReaching{false};        
    };
    suppressStruct cur_suppressStruct;

    // Variables to record current valid ranges
    std::mutex validRangeMutex;
    struct distPathState {
        float dist_valid_onPath;
        float dist_valid_devPath;
    };
    struct ValidWay { 
        float toLeft;
        float toRight;
        float toRear;
    };
    struct validRangeStruct{
        std::vector<distPathState> distPathstate_vec;
        ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
        bool on_GoTO_MODE = false;
        bool on_TURNING_MODE = false;
        float preFront_togo{-1.0f};
    };
    validRangeStruct cur_validRangeStruct;

    // Variables for stucks escape
    std::mutex stuckEscapeMutex;
    struct stuckEscapeStruct{
        int nFrontReachingTimer = 0;
        int nStuckEscapeCount = 0;
        float pre_front = -1.0f;
        float front_dev = -1.0f;
    };
    stuckEscapeStruct cur_stuckEscapeStruct;
    
    // Variables for static obstacles avoidance
    std::mutex staticObsMutex;
    struct preDist {
        float left;
        float right;
    };
    struct ReachEndState {
        bool reachFront;
        bool reachLeft;
        bool reachRight;
        bool reachRear;
    };
    struct staticObsStruct{
        float cur_distToMove{0.0f};
        int time_toMove = 1;
        preDist cur_preDist = {-1.0f, -1.0f};
        ReachEndState cur_reachEndState = { false, false, false, false };
        bool staticDodgeLeft = false;
        bool staticDodgeRight = false;
        bool isCloseToStaticObs = false;        
        int nObsStaticCount = 0;
        double ObsStaticElapsed = 0.0f;
        std::chrono::high_resolution_clock::time_point obsStaticStartTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point obsStaticEndTime = std::chrono::high_resolution_clock::now();
    };
    staticObsStruct cur_staticObsStruct;

    // Variables for dynamic obstacles avoidance
    std::mutex dynamicObsMutex;
    enum DodgeType {
        DODGE_STOP,
        DODGE_LEFT,
        DODGE_RIGHT,
        DODGE_REAR,
        DODGE_UP,
        DODGE_NONE
    };            
    enum localDodgeType {
        LOCAL_DODGE_LEFT,
        LOCAL_DODGE_RIGHT
    };
    struct obsState {
        float dist_obs;
        float aimDirection_obs;
    };
    struct dynamicObsStruct{
        float dodgeDist{0.0f};
        float dodgeDist_UP{0.0f};
        bool has_dodgeToRear = false;
        DodgeType cur_dodgeType = DODGE_NONE;
        obsState cur_obsState = { -1.0f, -1.0f };
        bool has_possibleInterrupt = false;
        bool has_possibleInterrupt_dynamic = false;
        bool has_InterruptNeedToReDo = false;
        bool has_InterruptNeedToReDo_dynamic = false;        
        int nObsDynamicCount = 0;
        double ObsDynamicElapsed = 0.0f;
        std::chrono::high_resolution_clock::time_point obsDynamicStartTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point obsDynamicEndTime = std::chrono::high_resolution_clock::now();
    };
    dynamicObsStruct cur_dynamicObsStruct;

    // Variables for front reaching
    std::mutex frontReachingMutex;
    struct pathReachingState {
        bool pathReadyToGo;
        bool pathOnGoing;
        float startFront;
    };
    struct frontReachingStruct{
        pathReachingState cur_pathReachingState = {false, false, -1.0f};        
        int nfrontReachingCount = 0;
        double FrontReachingElapsed = 0.0f;
        std::chrono::high_resolution_clock::time_point frontReachingStartTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point frontReachingEndTime = std::chrono::high_resolution_clock::now();
    };
    frontReachingStruct cur_frontReachingStruct;

    // Variables for target finding
    std::mutex targetFindingMutex;
    struct targetCheckState {
        bool aimTurnStarted;
        bool pointToTarget;
        bool turnStarted;
        float startAngle;
        float cur_aimDiff;
        float ang_toTurn;
        float targetAngle;
        float oriAimDirection;
    };
    struct targetFindingStruct{
        targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f, -1.0f};
        float start_turning_angle{0.0f};    
        float dist_to_reach;
        float aimDirection_to_reach;   
        int nTargetFindingCount = 0;
        double TargetFindingElapsed = 0.0f;
        std::chrono::high_resolution_clock::time_point targetFindingStartTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point targetFindingEndTime = std::chrono::high_resolution_clock::now();
    };
    targetFindingStruct cur_targetFindingStruct;

    // Variables for homing
    // float homing_batterythreshold = 3.35f;
    // float homing_batterythreshold = 2.5f;

    // Variables for looking around
    std::mutex lookAroundMutex;
    struct angleFrontState {
        float angle;
        float front;
    };    
    struct lookAroundState {
        bool clearPathCheckStarted;
        bool turnStarted;
        bool smallToBig;
        float preAngle;
        float startAngle;
        float targetAngle;
        int nTimer;
    };
    struct lookAroundStruct{
        std::vector<angleFrontState> angleFrontState_vec;
        lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
        float ori_front{0.0f};        
        int nlookAroundCount = 0;
        double LookAroundElapsed = 0.0f;
        std::chrono::high_resolution_clock::time_point lookAroundStartTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
    };
    lookAroundStruct cur_lookAroundStruct;

    // Timer to record time of each behaviour
    auto taskStartTime = std::chrono::high_resolution_clock::now();
    auto taskEndTime = std::chrono::high_resolution_clock::now();
    std::atomic<bool> isTerminateThread{false};

    // Take off first
    while ( cur_sensorReadStruct.cur_state_battery_state <= takeoff_batterythreshold ){
        std::cout <<" Battery is too low for taking off..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Takeoff(od4, 1.0f, 3);
    taskStartTime = std::chrono::high_resolution_clock::now();

    // Thread for valid direction check
    std::thread ValidRangeCheckTask([od4, &isTerminateThread,
                                    &cur_sensorReadStruct,
                                    &validRangeMutex, &cur_validRangeStruct,
                                    &lookAroundMutex, &cur_lookAroundStruct]() {
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Variables for sensor read
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for looking around
            std::vector<angleFrontState> angleFrontState_vec;
            lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
            float ori_front{0.0f};
            int nlookAroundCount = 0;
            double LookAroundElapsed = 0.0f;
            auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
            auto lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(lookAroundMutex);
                angleFrontState_vec = cur_lookAroundStruct.angleFrontState_vec;
                cur_lookAroundState = cur_lookAroundStruct.cur_lookAroundState;
                ori_front = cur_lookAroundStruct.ori_front;
                nlookAroundCount = cur_lookAroundStruct.nlookAroundCount;
                LookAroundElapsed = cur_lookAroundStruct.LookAroundElapsed;
                lookAroundStartTime = cur_lookAroundStruct.lookAroundStartTime;
                lookAroundEndTime = cur_lookAroundStruct.lookAroundEndTime;
            }

            /*
                Valid direction check
                - Check from the range recording array
            */
            // Recheck current dist path vector if in turning mode
            // Clear the vectors in case of memory problems
            if ( on_TURNING_MODE && angleFrontState_vec.size() > 0 ){
                // Try to refresh the rear distance
                float rearDist{-1.0f};
                float angMin = std::abs( std::atan2( 0.1f, front ) );
                for ( const auto& pair : angleFrontState_vec ){
                    // std::cout <<" Current angle to check, angle: " << pair.angle << ", front: " << pair.front << std::endl; 
                    float angDev = std::abs( angleDifference( pair.angle, cur_state_yaw ) );
                    if ( angDev >= 135.0f / 180.0f * M_PI ){
                        if ( pair.front * std::sin( angDev ) > 0.1f ){
                            continue;
                        }
                    }  
                    else if ( angDev < 135.0f / 180.0f * M_PI ){
                        continue;
                    }

                    if ( std::abs( pair.front * std::cos( angDev ) ) < rearDist || rearDist == -1.0f ){
                        rearDist = std::abs( pair.front * std::cos( angDev ) );
                    }                   
                }
                cur_validWay.toRear = rearDist;

                // Record path related information
                ori_front = front;
                preFront_togo = front;
                if ( distPathstate_vec.size() > 0 ){
                    distPathstate_vec.clear();
                }
                for ( const auto& pair : angleFrontState_vec ){
                    float angDev = angleDifference( cur_state_yaw, pair.angle );
                    // std::cout <<" Current angle difference: " << angDev << ", front: " <<pair.front << ", current angle: " << cur_state_yaw << std::endl; 
                    // std::cout <<" Cosine(On Path): " << pair.front*std::cos( angDev ) << ", Sine(Dev Path): " << pair.front*std::sin( angDev ) << std::endl; 
                    distPathState pstate = { pair.front*std::cos( angDev ), pair.front*std::sin( angDev ) }; 
                    distPathstate_vec.insert( distPathstate_vec.begin(), pstate );              
                }

                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(validRangeMutex);
                    cur_validRangeStruct.distPathstate_vec = distPathstate_vec;
                    cur_validRangeStruct.cur_validWay.toRear = cur_validWay.toRear;
                    cur_validRangeStruct.preFront_togo = preFront_togo;
                }
                {
                    std::lock_guard<std::mutex> lck(lookAroundMutex);
                    cur_lookAroundStruct.ori_front = ori_front;
                }
            }

            // Refresh valid left/right/rear on the way
            if ( distPathstate_vec.size() > 0 && on_GoTO_MODE ){
                float cur_dist_onPath = ori_front - front;
                bool hasFoundOnPath = false;
                for ( auto& state : distPathstate_vec ){
                    if ( std::abs( state.dist_valid_onPath - cur_dist_onPath ) > 0.01f ){
                        continue;
                    }
                    // std::cout << "Current path progress: " << cur_dist_onPath << " with original devPath:" << state.dist_valid_devPath << ", left: " << left << ", right: " << right << std::endl;
                    // std::cout <<" Distance to be modified, right: " << right << ", left: " << left << ", path progress: " << std::abs( state.dist_valid_onPath - cur_dist_onPath ) << std::endl; 

                    hasFoundOnPath = true;
                    if ( state.dist_valid_devPath <= 0.0f ){
                        state.dist_valid_devPath = -right;
                    }
                    else{
                        state.dist_valid_devPath = left; 
                    }
                }

                if ( hasFoundOnPath == false ){
                    distPathState pstate = { cur_dist_onPath, -right }; 
                    distPathstate_vec.insert( distPathstate_vec.begin(), pstate ); 
                    pstate = { cur_dist_onPath, left }; 
                    distPathstate_vec.insert( distPathstate_vec.begin(), pstate ); 
                }

                // Refresh valid to rear
                if ( cur_validWay.toRear == -1.0f ){
                    cur_validWay.toRear = 0.0f;
                }
                cur_validWay.toRear += preFront_togo - front;
                preFront_togo = front;
                // std::cout <<" Refresh distpath on goto mode with rear: " << cur_validWay.toRear << std::endl;
                
                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(validRangeMutex);
                    cur_validRangeStruct.distPathstate_vec = distPathstate_vec;
                    cur_validRangeStruct.cur_validWay.toRear = cur_validWay.toRear;
                    cur_validRangeStruct.preFront_togo = preFront_togo;
                }
            }

            // Start valid way checking
            if ( distPathstate_vec.size() > 0 ){
                cur_validWay.toLeft = -1.0f;
                cur_validWay.toRight = -1.0f;

                float cur_dist_onPath = ori_front - front;
                float toLeft{4.0f};
                float toRight{4.0f};
                for ( auto& state : distPathstate_vec ){
                    // std::cout <<" Current front diff, front: " << front << ", original front: "<< ori_front << std::endl; 
                    // std::cout <<" Distance to check, state on path:" << state.dist_valid_onPath << std::endl; 
                    
                    // While some path in Crazyflie range
                    if ( std::abs( state.dist_valid_onPath - cur_dist_onPath ) > 0.1f ){
                        continue;
                    }

                    if ( state.dist_valid_devPath > 0.0f ){
                        if ( std::abs( state.dist_valid_devPath ) < toLeft ){
                            toLeft = std::abs( state.dist_valid_devPath );
                        }
                    }
                    else{
                        if ( std::abs( state.dist_valid_devPath ) < toRight ){
                            toRight = std::abs( state.dist_valid_devPath );
                        }
                    }
                }

                // Set current valid way to the state
                if ( toLeft != 4.0f ){
                    cur_validWay.toLeft = toLeft;
                }
                else{
                    cur_validWay.toLeft = left;
                }
                if ( toRight != 4.0f ){
                    cur_validWay.toRight = toRight;
                }
                else{
                    cur_validWay.toRight = right;
                }
                // std::cout <<" Valid way checking start... with left " << cur_validWay.toLeft << ", with right: " << cur_validWay.toRight << ", with rear: " << cur_validWay.toRear << ", and current angle: " << cur_state_yaw << std::endl;
                
                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(validRangeMutex);
                    cur_validRangeStruct.cur_validWay.toLeft = cur_validWay.toLeft;
                    cur_validRangeStruct.cur_validWay.toRight = cur_validWay.toRight;
                }
            }
        }              
    });

    std::thread StuckEscapeTask([od4, &isTerminateThread,
                                 &cur_sensorReadStruct,
                                 &stuckEscapeMutex, &cur_stuckEscapeStruct,
                                 &dynamicObsMutex, &cur_dynamicObsStruct](){
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Variables here            
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            int nFrontReachingTimer = 0;
            int nStuckEscapeCount = 0;
            float pre_front = -1.0f;
            float front_dev = -1.0f;
            {
                std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                nFrontReachingTimer = cur_stuckEscapeStruct.nFrontReachingTimer;
                nStuckEscapeCount = cur_stuckEscapeStruct.nStuckEscapeCount;
                pre_front = cur_stuckEscapeStruct.pre_front;
                front_dev = cur_stuckEscapeStruct.front_dev;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }           

            /*
                Stucks Escape for 10 secs
            */
            if ( pre_front == -1.0f ){
                pre_front = front;
                nFrontReachingTimer += 1;

                // Set variables back            
                {
                    std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                    cur_stuckEscapeStruct.nFrontReachingTimer = nFrontReachingTimer;
                    cur_stuckEscapeStruct.pre_front = pre_front;
                } 
            }
            else if ( nFrontReachingTimer <= 1000){
                float dev = std::abs( pre_front - front );
                if ( dev > front_dev ){
                    front_dev = dev;   

                    // Set variables back        
                    {
                        std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                        cur_stuckEscapeStruct.front_dev = front_dev;
                    } 
                }
                nFrontReachingTimer += 1;

                // Set variables back            
                {
                    std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                    cur_stuckEscapeStruct.nFrontReachingTimer = nFrontReachingTimer;
                } 
            }
            else{
                if ( front_dev <= 0.1f ){
                    has_InterruptNeedToReDo = true;
                    std::cout << "Stucks at some positions, try to escape by redo..." << std::endl;
                    nStuckEscapeCount += 1;

                    // Set variables back            
                    {
                        std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                        cur_stuckEscapeStruct.nStuckEscapeCount = nStuckEscapeCount;
                    } 
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    }  
                }
                nFrontReachingTimer = 0;
                pre_front = -1.0f;
                front_dev = -1.0f;

                // Set variables back            
                {
                    std::lock_guard<std::mutex> lck(stuckEscapeMutex);
                    cur_stuckEscapeStruct.nFrontReachingTimer = nFrontReachingTimer;
                    cur_stuckEscapeStruct.pre_front = pre_front;
                    cur_stuckEscapeStruct.front_dev = front_dev;
                } 
            }
        }
    });

    std::thread StaticObsDodgeTask([od4, &isTerminateThread,
                                    &cur_sensorReadStruct, &cur_constVarStruct,
                                    &suppressMutex, &cur_suppressStruct,
                                    &validRangeMutex, &cur_validRangeStruct,
                                    &staticObsMutex, &cur_staticObsStruct,
                                    &dynamicObsMutex, &cur_dynamicObsStruct](){
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Variables here        
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for constant
            float safe_endreach_ultimate_dist{0.0f};
            float safe_endreach_dist{0.0f};
            float safe_endreach_LR_dist{0.0f};
            int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
            {
                safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
                safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
                safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
                nTargetCount = cur_constVarStruct.nTargetCount;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for static obstacles avoidance
            float cur_distToMove{0.0f};
            int time_toMove = 1;
            preDist cur_preDist = {-1.0f, -1.0f};
            ReachEndState cur_reachEndState = { false, false, false, false };
            bool staticDodgeLeft = false;
            bool staticDodgeRight = false;
            bool isCloseToStaticObs = false;
            int nObsStaticCount = 0;
            double ObsStaticElapsed = 0.0f;
            auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
            auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(staticObsMutex);
                cur_distToMove = cur_staticObsStruct.cur_distToMove;
                time_toMove = cur_staticObsStruct.time_toMove;
                cur_preDist = cur_staticObsStruct.cur_preDist;
                cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
                staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
                staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
                isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
                nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
                ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
                obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
                obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }    

            /*
                Obstacle Avoidance for walls or static obstacles
                - Get the range from rangefinder
                - Check whether some ranges reach ends
                - Can be tuned:
                -- safe distance
                -- delay time
            */
            // Check if the crazyflie stucks at some points
            float safe_dist{0.0f};
            if ( front >= 1.0f )
                safe_dist = safe_endreach_dist + front / 4 + 0.35f;
            else
                safe_dist = safe_endreach_dist;
            if ( front <= safe_endreach_ultimate_dist ){
                std::cout <<" Front is super close to something..." << std::endl;
                if ( cur_reachEndState.reachFront == false ){
                    nObsStaticCount += 1;
                    obsStaticStartTime = std::chrono::high_resolution_clock::now();
                    cur_reachEndState.reachFront = true; 
                    on_GoTO_MODE = false;
                    has_InterruptNeedToReDo = true;
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                    }
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_reachEndState.reachFront = cur_reachEndState.reachFront;
                        cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                        cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                    } 
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    } 
                }
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = true;
                }
                if ( rear >= 0.2f + safe_endreach_ultimate_dist ){
                    Goto(od4, - 0.2f * std::cos( cur_state_yaw ), - 0.2f * std::sin( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying rear to dodge
                }
                else{
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }              
                }
                continue;
            }
            else if ( front <= safe_dist ){
                if ( on_GoTO_MODE ){
                    std::cout <<" Front end meets limit." << std::endl;
                    if ( cur_reachEndState.reachFront == false ){
                        nObsStaticCount += 1;
                        obsStaticStartTime = std::chrono::high_resolution_clock::now();
                        cur_reachEndState.reachFront = true; 
                        on_GoTO_MODE = false;  
                
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(validRangeMutex);
                            cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                        }
                        {
                            std::lock_guard<std::mutex> lck(staticObsMutex);
                            cur_staticObsStruct.cur_reachEndState.reachFront = cur_reachEndState.reachFront;
                            cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                            cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                        }                   
                    }
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = true;
                    }
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }
                }           
            }
            else{
                if ( cur_reachEndState.reachFront ){
                    obsStaticEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = obsStaticEndTime - obsStaticStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticEndTime)
                    );

                    std::cout <<" Obs static front complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    ObsStaticElapsed += elapsed.count();
                    std::cout <<" , average front static obs elapsed: " << ObsStaticElapsed / nObsStaticCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has close to front static obs for " << nObsStaticCount << " times" << std::endl;
                    cur_reachEndState.reachFront = false;
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.obsStaticEndTime = obsStaticEndTime;
                        cur_staticObsStruct.ObsStaticElapsed = ObsStaticElapsed;
                        cur_staticObsStruct.cur_reachEndState.reachFront = cur_reachEndState.reachFront;
                    }                     
                }
                
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = false;
                    cur_suppressStruct.isSupressFrontReaching = false;
                }
            }

            if ( left <= safe_endreach_ultimate_dist ){
                std::cout <<" Left is super close to something..." << std::endl;
                if ( cur_reachEndState.reachLeft == false ){
                    nObsStaticCount += 1;
                    obsStaticStartTime = std::chrono::high_resolution_clock::now();
                    cur_reachEndState.reachLeft = true; 
                    has_InterruptNeedToReDo = true;             
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_reachEndState.reachLeft = cur_reachEndState.reachLeft;
                        cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                        cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                    } 
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    } 
                }
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = true;
                }
                if ( right >= 0.2f + safe_endreach_ultimate_dist ){
                    Goto(od4, 0.2f * std::sin( cur_state_yaw ), - 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying rear to dodge
                }
                else{
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction    
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }            
                }    
                continue;
            }
            else if ( left <= safe_endreach_LR_dist ){
                if ( on_GoTO_MODE == false ){
                    cur_preDist.left = -1.0f;           
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_preDist.left = cur_preDist.left;
                    } 
                }
                else{
                    if ( cur_preDist.left == -1.0f || cur_preDist.left <= left ){
                        cur_preDist.left = left;
                        std::cout <<" Record left: "<< cur_preDist.left << std::endl;
                
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(staticObsMutex);
                            cur_staticObsStruct.cur_preDist.left = cur_preDist.left;
                        } 
                        continue;
                    }
                    else{
                        std::cout <<" Cur left: "<< left << std::endl;
                        std::cout <<" Cur toRight: "<< cur_validWay.toRight << std::endl;
                        if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f ){
                            std::cout <<" Left end meets limit with right direction dodge..." << std::endl;
                            if ( staticDodgeLeft == false ){
                                nObsStaticCount += 1;
                                has_possibleInterrupt = true;
                                obsStaticStartTime = std::chrono::high_resolution_clock::now();
                                staticDodgeLeft = true;                                
                
                                // Set variables back
                                {
                                    std::lock_guard<std::mutex> lck(staticObsMutex);
                                    cur_staticObsStruct.staticDodgeLeft = staticDodgeLeft;
                                    cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                                    cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                                } 
                                {
                                    std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                    cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                                } 
                            }
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = true;
                            }
                            Goto(od4, 0.2f * std::sin( cur_state_yaw ), - 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge                        
                            continue;
                        }
                        else{
                            std::cout <<" Left end meets limit." << std::endl;
                            if ( cur_reachEndState.reachLeft == false ){
                                nObsStaticCount += 1;
                                cur_reachEndState.reachLeft = true;
                                obsStaticStartTime = std::chrono::high_resolution_clock::now();                           
                
                                // Set variables back
                                {
                                    std::lock_guard<std::mutex> lck(staticObsMutex);
                                    cur_staticObsStruct.cur_reachEndState.reachLeft = cur_reachEndState.reachLeft;
                                    cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                                    cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                                } 
                            }
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = true;
                            }
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = false;
                                cur_suppressStruct.isSupressFrontReaching = true;
                            }
                        }
                    }                
                }
            }
            else{ 
                if ( staticDodgeLeft || cur_reachEndState.reachLeft ){
                    obsStaticEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = obsStaticEndTime - obsStaticStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticEndTime)
                    );

                    std::cout <<" Obs static left complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    ObsStaticElapsed += elapsed.count();
                    std::cout <<" , average left static obs elapsed: " << ObsStaticElapsed / nObsStaticCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has close to left static obs for " << nObsStaticCount << " times" << std::endl;
                    staticDodgeLeft = false;
                    cur_reachEndState.reachLeft = false;

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.obsStaticEndTime = obsStaticEndTime;
                        cur_staticObsStruct.ObsStaticElapsed = ObsStaticElapsed;
                        cur_staticObsStruct.staticDodgeLeft = staticDodgeLeft;
                        cur_staticObsStruct.cur_reachEndState.reachLeft = cur_reachEndState.reachLeft;
                    } 
                }
                
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = false;
                    cur_suppressStruct.isSupressFrontReaching = false;
                }
            }

            if ( right <= safe_endreach_ultimate_dist ){
                std::cout <<" Right is super close to something..." << std::endl;
                if ( cur_reachEndState.reachRight == false ){
                    nObsStaticCount += 1;
                    obsStaticStartTime = std::chrono::high_resolution_clock::now();
                    cur_reachEndState.reachRight = true; 
                    has_InterruptNeedToReDo = true;          
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_reachEndState.reachRight = cur_reachEndState.reachRight;
                        cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                        cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                    } 
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    } 
                }
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = true;
                }
                if ( left >= 0.2f + safe_endreach_ultimate_dist ){
                    Goto(od4, - 0.2f * std::sin( cur_state_yaw ), 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying rear to dodge
                }
                else{
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }              
                }
                continue;
            }
            else if ( right <= safe_endreach_LR_dist ){
                if ( on_GoTO_MODE == false ){
                    cur_preDist.right = -1.0f;    
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_preDist.right = cur_preDist.right;
                    } 
                }
                else{
                    if ( cur_preDist.right == -1.0f || cur_preDist.right <= right ){
                        cur_preDist.right = right;
                        std::cout <<" Record right: "<< cur_preDist.right << std::endl;    
                
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(staticObsMutex);
                            cur_staticObsStruct.cur_preDist.right = cur_preDist.right;
                        } 
                        continue;
                    }
                    else{
                        std::cout <<" Cur right: "<< right << std::endl;
                        std::cout <<" Cur toLeft: "<< cur_validWay.toLeft << std::endl;
                        if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f ){
                            std::cout <<" Right end meets limit with left direction dodge..." << std::endl;
                            if ( staticDodgeRight == false ){
                                staticDodgeRight = true;
                                has_possibleInterrupt = true;
                                obsStaticStartTime = std::chrono::high_resolution_clock::now();
                                nObsStaticCount += 1;       
                
                                // Set variables back
                                {
                                    std::lock_guard<std::mutex> lck(staticObsMutex);
                                    cur_staticObsStruct.staticDodgeRight = staticDodgeRight;
                                    cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                                    cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                                } 
                                {
                                    std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                    cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                                } 
                            }
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = true;
                            }
                            Goto(od4, - 0.2f * std::sin( cur_state_yaw ), 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                            continue;
                        }
                        else{
                            std::cout <<" Right end meets limit." << std::endl;
                            if ( cur_reachEndState.reachRight == false ){
                                cur_reachEndState.reachRight = true;
                                obsStaticStartTime = std::chrono::high_resolution_clock::now();
                                nObsStaticCount += 1;    
                
                                // Set variables back
                                {
                                    std::lock_guard<std::mutex> lck(staticObsMutex);
                                    cur_staticObsStruct.cur_reachEndState.reachRight = cur_reachEndState.reachRight;
                                    cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                                    cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                                } 
                            }
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = true;
                            }
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isObsStaticDominating = false;
                                cur_suppressStruct.isSupressFrontReaching = true;
                            }
                        }
                    }
                }
            }
            else{
                if ( staticDodgeRight || cur_reachEndState.reachRight ){
                    obsStaticEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = obsStaticEndTime - obsStaticStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticEndTime)
                    );

                    std::cout <<" Obs static right complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    ObsStaticElapsed += elapsed.count();
                    std::cout <<" , average right static obs elapsed: " << ObsStaticElapsed / nObsStaticCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has close to right static obs for " << nObsStaticCount << " times" << std::endl;
                    staticDodgeRight = false;
                    cur_reachEndState.reachRight = false;                      
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.obsStaticEndTime = obsStaticEndTime;
                        cur_staticObsStruct.ObsStaticElapsed = ObsStaticElapsed;
                        cur_staticObsStruct.staticDodgeRight = staticDodgeRight;
                        cur_staticObsStruct.cur_reachEndState.reachRight = cur_reachEndState.reachRight;
                    } 
                }

                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = false;
                    cur_suppressStruct.isSupressFrontReaching = false;
                }
            }

            if ( rear <= safe_endreach_ultimate_dist ){
                std::cout <<" Rear is super close to something..." << std::endl;
                if ( cur_reachEndState.reachRear == false ){
                    nObsStaticCount += 1;
                    obsStaticStartTime = std::chrono::high_resolution_clock::now();
                    cur_reachEndState.reachRear = true; 
                    has_InterruptNeedToReDo = true;   
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                        cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                        cur_staticObsStruct.cur_reachEndState.reachRear = cur_reachEndState.reachRear;
                    } 
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    } 
                }
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = true;
                }
                if ( rear >= 0.2f + safe_endreach_ultimate_dist ){
                    Goto(od4, 0.2f * std::cos( cur_state_yaw ), 0.2f * std::sin( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying rear to dodge
                }
                else{
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction    
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }            
                }
                continue;
            }
            else if ( rear <= safe_endreach_dist ){
                if ( cur_dodgeType != DODGE_NONE ){
                    std::cout <<" Rear end meets limit." << std::endl;
                    if ( cur_reachEndState.reachRear == false ){
                        nObsStaticCount += 1;
                        obsStaticStartTime = std::chrono::high_resolution_clock::now();
                        cur_reachEndState.reachRear = true;
                        on_GoTO_MODE = false;
                
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(staticObsMutex);
                            cur_staticObsStruct.nObsStaticCount = nObsStaticCount;
                            cur_staticObsStruct.obsStaticStartTime = obsStaticStartTime;
                            cur_staticObsStruct.cur_reachEndState.reachRear = cur_reachEndState.reachRear;
                        } 
                        {
                            std::lock_guard<std::mutex> lck(validRangeMutex);
                            cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                        } 
                    }
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = true;
                    }                    
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsStaticDominating = false;
                        cur_suppressStruct.isSupressFrontReaching = true;
                    }
                }          
            }
            else{
                if ( cur_reachEndState.reachRear ){
                    obsStaticEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = obsStaticEndTime - obsStaticStartTime;
                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsStaticEndTime)
                    );

                    std::cout <<" Obs static rear complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    ObsStaticElapsed += elapsed.count();
                    std::cout <<" , average rear static obs elapsed: " << ObsStaticElapsed / nObsStaticCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has close to rear static obs for " << nObsStaticCount << " times" << std::endl;
                    cur_reachEndState.reachRear = false;
                
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.obsStaticEndTime = obsStaticEndTime;
                        cur_staticObsStruct.ObsStaticElapsed = ObsStaticElapsed;
                        cur_staticObsStruct.cur_reachEndState.reachRear = cur_reachEndState.reachRear;
                    } 
                }

                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsStaticDominating = false;
                    cur_suppressStruct.isSupressFrontReaching = false;
                }
            }    
        }        
    });

    std::thread DynamicObsDodgeTask([od4, &isTerminateThread,
                                     &suppressMutex, &cur_suppressStruct,
                                     &cur_sensorReadStruct, 
                                     &cur_constVarStruct,
                                     &validRangeMutex, &cur_validRangeStruct,
                                     &staticObsMutex, &cur_staticObsStruct,
                                     &dynamicObsMutex, &cur_dynamicObsStruct](){
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ( cur_suppressStruct.isObsStaticDominating ){
                // std::cout << "Dynamic obs task being suppressed!" << std::endl;
                continue;
            }   // Wait until the domination change to the dynamic obstacle domination

            // std::cout << "I am in the dynamic obs task!" << std::endl;
          
            // Variables here        
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for constant
            float safe_endreach_ultimate_dist{0.0f};
            float safe_endreach_dist{0.0f};
            float safe_endreach_LR_dist{0.0f};
            int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
            {
                safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
                safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
                safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
                nTargetCount = cur_constVarStruct.nTargetCount;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for static obstacles avoidance
            float cur_distToMove{0.0f};
            int time_toMove = 1;
            preDist cur_preDist = {-1.0f, -1.0f};
            ReachEndState cur_reachEndState = { false, false, false, false };
            bool staticDodgeLeft = false;
            bool staticDodgeRight = false;
            bool isCloseToStaticObs = false;
            int nObsStaticCount = 0;
            double ObsStaticElapsed = 0.0f;
            auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
            auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(staticObsMutex);
                cur_distToMove = cur_staticObsStruct.cur_distToMove;
                time_toMove = cur_staticObsStruct.time_toMove;
                cur_preDist = cur_staticObsStruct.cur_preDist;
                cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
                staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
                staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
                isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
                nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
                ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
                obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
                obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }   

            /*
                Obstacle Avoidance for dynamic obstacles
                - Check the obstacle in the vision
                - Use current valid way to dodge away from the obstacle
                - Can be tuned:
                -- safe distance
                -- delay time
            */
            if ( dist_obs > -1.0f ){    // Means that some obstacles approach
                std::cout <<" Has dynamic obstacles with to left: " << cur_validWay.toLeft << ", to right: " << cur_validWay.toRight << ", to rear: " << cur_validWay.toRear << ", obs on path: " << dist_obs * std::cos( aimDirection_obs ) << ", dist: " << dist_obs << ", aimDirection: " << aimDirection_obs << std::endl; 
                if ( cur_dodgeType == DODGE_NONE ){
                    nObsDynamicCount += 1;
                    obsDynamicStartTime = std::chrono::high_resolution_clock::now();
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isObsDynamicDominating = true;
                    }
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.nObsDynamicCount = nObsDynamicCount;
                        cur_dynamicObsStruct.obsDynamicStartTime = obsDynamicStartTime;
                    }
                }

                if ( dist_obs * std::cos( aimDirection_obs ) <= 135.0f ){    
                    std::cout <<" Obstacle gets too close..." << std::endl;            
                    if ( aimDirection_obs < 0.0f ){
                        // If left side has valid way, dodge to it
                        if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f ){
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                            std::cout <<" Try to dodge to the left..." << std::endl;
                            Goto(od4, - 0.2f * std::sin( cur_state_yaw ), 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            dodgeDist += 0.2f;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_LEFT;                            
                            
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.dodgeDist = dodgeDist;
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            continue;
                        }
                        else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                            std::cout <<" Try to dodge to the rear..." << std::endl;
                            cur_distToMove = cur_validWay.toRear;
                            if ( cur_distToMove >= 1.0f )
                                time_toMove = 4;
                            else
                                time_toMove = 5;
                            Goto(od4, - cur_distToMove * std::cos( cur_state_yaw ), - cur_distToMove * std::sin( cur_state_yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                            on_GoTO_MODE = true;
                            cur_dodgeType = DODGE_REAR;
                            has_dodgeToRear = true;
                            has_InterruptNeedToReDo_dynamic = true;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(staticObsMutex);
                                cur_staticObsStruct.cur_distToMove = cur_distToMove;
                                cur_staticObsStruct.time_toMove = time_toMove;
                            } 
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                                cur_dynamicObsStruct.has_dodgeToRear = has_dodgeToRear;
                                cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                            }  
                            continue;
                        }
                        else{
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop
                            std::cout <<" No where to go but stop current action..." << std::endl;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_STOP;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            } 
                        }
                    }
                    else{
                        // If right side has valid way, dodge to it
                        if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f ){
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                            std::cout <<" Try to dodge to the right..." << std::endl;
                            Goto(od4, 0.2f * std::sin( cur_state_yaw ), - 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            dodgeDist -= 0.2f;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_RIGHT;                         
                            
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.dodgeDist = dodgeDist;
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            continue;
                        }
                        else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                            std::cout <<" Try to dodge to the rear..." << std::endl;
                            cur_distToMove = cur_validWay.toRear;
                            if ( cur_distToMove >= 1.0f )
                                time_toMove = 4;
                            else
                                time_toMove = 5;
                            Goto(od4, - cur_distToMove * std::cos( cur_state_yaw ), - cur_distToMove * std::sin( cur_state_yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                            on_GoTO_MODE = true;
                            cur_dodgeType = DODGE_REAR;
                            has_dodgeToRear = true;
                            has_InterruptNeedToReDo_dynamic = true;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(staticObsMutex);
                                cur_staticObsStruct.cur_distToMove = cur_distToMove;
                                cur_staticObsStruct.time_toMove = time_toMove;
                            } 
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                                cur_dynamicObsStruct.has_dodgeToRear = has_dodgeToRear;
                                cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                            }  
                            continue;
                        }
                        else{
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop
                            std::cout <<" No where to go but stop current action..." << std::endl;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_STOP;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            } 
                        }
                    }
                }

                if ( cur_dodgeType == DODGE_NONE ){
                    // Stop first and record current dist_obs
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);
                    cur_obsState.dist_obs = dist_obs;
                    cur_obsState.aimDirection_obs = aimDirection_obs;
                    cur_dodgeType = DODGE_STOP;

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.cur_obsState.dist_obs = cur_obsState.dist_obs;
                        cur_dynamicObsStruct.cur_obsState.aimDirection_obs = cur_obsState.aimDirection_obs;
                        cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                    }  
                    continue;
                }

                // Check if the obstacle is on the left or right side   
                if ( std::abs( dist_obs * std::cos( aimDirection_obs ) - cur_obsState.dist_obs * std::cos( cur_obsState.aimDirection_obs ) ) > 13.0f && std::abs( angleDifference(aimDirection_obs, cur_obsState.aimDirection_obs) ) <= 10.0f / 180.0f * M_PI && cur_obsState.dist_obs >= dist_obs ){
                    localDodgeType cur_localDodgeType = LOCAL_DODGE_LEFT;
                    if ( aimDirection_obs >= 0.0f && cur_obsState.aimDirection_obs >= 0.0f ){
                        std::cout <<" Both in left side... " << std::endl; 
                        if ( aimDirection_obs >= cur_obsState.aimDirection_obs ){
                            cur_localDodgeType = LOCAL_DODGE_RIGHT;  
                            std::cout <<" Current angle larger than before... " << std::endl; 
                        }                  
                    }
                    else if ( aimDirection_obs >= 0.0f && cur_obsState.aimDirection_obs < 0.0f ){
                        std::cout <<" In different sides... " << std::endl; 
                        cur_localDodgeType = LOCAL_DODGE_RIGHT;
                    }
                    else if ( aimDirection_obs < 0.0f && cur_obsState.aimDirection_obs < 0.0f ){
                        std::cout <<" Both in right side... " << std::endl; 
                        if ( aimDirection_obs >= cur_obsState.aimDirection_obs ){
                            cur_localDodgeType = LOCAL_DODGE_RIGHT;  
                            std::cout <<" Current angle larger than before... " << std::endl; 
                        }    
                    }            
                    
                    if ( cur_localDodgeType == LOCAL_DODGE_LEFT ){
                        // If left side has valid way, dodge to it
                        if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f ){
                            // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                            std::cout <<" Try to dodge to the left..." << std::endl;
                            Goto(od4, - 0.2f * std::sin( cur_state_yaw ), 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            dodgeDist += 0.2f;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_LEFT;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.dodgeDist = dodgeDist;
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }  
                            continue;
                        }
                        else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                            // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                            std::cout <<" Try to dodge to the rear..." << std::endl;
                            cur_distToMove = cur_validWay.toRear;
                            if ( cur_distToMove >= 1.0f )
                                time_toMove = 4;
                            else
                                time_toMove = 5;
                            Goto(od4, - cur_distToMove * std::cos( cur_state_yaw ), - cur_distToMove * std::sin( cur_state_yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                            on_GoTO_MODE = true;
                            cur_dodgeType = DODGE_REAR;
                            has_dodgeToRear = true;
                            has_InterruptNeedToReDo_dynamic = true;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(staticObsMutex);
                                cur_staticObsStruct.cur_distToMove = cur_distToMove;
                                cur_staticObsStruct.time_toMove = time_toMove;
                            } 
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                                cur_dynamicObsStruct.has_dodgeToRear = has_dodgeToRear;
                                cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                            }  
                            continue;
                        }
                        else{
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop
                            std::cout <<" No where to go but stop current action..." << std::endl;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_STOP;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }  
                            continue;
                        }
                    }
                    else{
                        // If right side has valid way, dodge to it
                        if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f ){
                            // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                            std::cout <<" Try to dodge to the right..." << std::endl;
                            Goto(od4, 0.2f * std::sin( cur_state_yaw ), - 0.2f * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            dodgeDist -= 0.2f;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_RIGHT;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.dodgeDist = dodgeDist;
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }  
                            continue;
                        }
                        else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                            // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                            std::cout <<" Try to dodge to the rear..." << std::endl;
                            cur_distToMove = cur_validWay.toRear;
                            if ( cur_distToMove >= 1.0f )
                                time_toMove = 4;
                            else
                                time_toMove = 5;
                            Goto(od4, - cur_distToMove * std::cos( cur_state_yaw ), - cur_distToMove * std::sin( cur_state_yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                            on_GoTO_MODE = true;
                            cur_dodgeType = DODGE_REAR;
                            has_dodgeToRear = true;
                            has_InterruptNeedToReDo_dynamic = true;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(staticObsMutex);
                                cur_staticObsStruct.cur_distToMove = cur_distToMove;
                                cur_staticObsStruct.time_toMove = time_toMove;
                            } 
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                                cur_dynamicObsStruct.has_dodgeToRear = has_dodgeToRear;
                                cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                            }  
                            continue;
                        }
                        else{
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop
                            std::cout <<" No where to go but stop current action..." << std::endl;
                            on_GoTO_MODE = false;
                            cur_dodgeType = DODGE_STOP;
                            
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                            }
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                            }  
                            continue;
                        }
                    }
                }

                // Record current dist and aimDirection for later use
                cur_obsState.dist_obs = dist_obs;
                cur_obsState.aimDirection_obs = aimDirection_obs;
                {
                    std::lock_guard<std::mutex> lck(dynamicObsMutex);
                    cur_dynamicObsStruct.cur_obsState.dist_obs = cur_obsState.dist_obs;
                    cur_dynamicObsStruct.cur_obsState.aimDirection_obs = cur_obsState.aimDirection_obs;
                } 
            }
            else if ( cur_dodgeType != DODGE_NONE ){
                // Go back to the original position while the obstacle is gone
                // if ( dodgeDist_UP != 0.0f ){
                //     std::cout <<" No obs, try to fly down..." << std::endl;
                //     Goto(od4, 0.0f, 0.0f, dodgeDist_UP, 0.0f, 0, 1, true);
                // }
                if ( dodgeDist != 0.0f ){
                    std::cout <<" No obs, try to fly back..." << std::endl;
                    Goto(od4, dodgeDist * std::sin( cur_state_yaw ), - dodgeDist * std::cos( cur_state_yaw ), 0.0f, 0.0f, 0, 1, true);
                }

                // Reset flags            
                // dodgeDist_UP = 0.0f;
                dodgeDist = 0.0f;
                cur_dodgeType = DODGE_NONE; 
                has_dodgeToRear = false;
                on_GoTO_MODE = false;
                if ( has_InterruptNeedToReDo_dynamic == false ){
                    has_possibleInterrupt_dynamic = true;
                }
                
                {
                    std::lock_guard<std::mutex> lck(suppressMutex);
                    cur_suppressStruct.isObsDynamicDominating = false;
                }

                obsDynamicEndTime = std::chrono::high_resolution_clock::now();
                const std::chrono::duration<double> elapsed = obsDynamicEndTime - obsDynamicStartTime;
                auto start_time_t = std::chrono::system_clock::to_time_t(
                    std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsDynamicStartTime)
                );
                auto end_time_t = std::chrono::system_clock::to_time_t(
                    std::chrono::time_point_cast<std::chrono::system_clock::duration>(obsDynamicEndTime)
                );

                std::cout <<" Obs dynamic complete with start time: " << std::ctime(&start_time_t) << std::endl;
                std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                ObsDynamicElapsed += elapsed.count();
                std::cout <<" , average dynamic obs elapsed: " << ObsDynamicElapsed / nObsDynamicCount << " seconds(s)" << std::endl;
                std::cout <<" , so far has close to dynamic obs for " << nObsDynamicCount << " times" << std::endl;
            
                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(validRangeMutex);
                    cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                }
                {
                    std::lock_guard<std::mutex> lck(dynamicObsMutex);
                    cur_dynamicObsStruct.dodgeDist = dodgeDist;
                    cur_dynamicObsStruct.cur_dodgeType = cur_dodgeType;
                    cur_dynamicObsStruct.has_dodgeToRear = has_dodgeToRear;
                    cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                    cur_dynamicObsStruct.obsDynamicEndTime = obsDynamicEndTime;
                    cur_dynamicObsStruct.ObsDynamicElapsed = ObsDynamicElapsed;
                }    
            }   
        }        
    });

    std::thread TargetFindingTask([od4, &isTerminateThread,
                                    &suppressMutex, &cur_suppressStruct,
                                    &cur_sensorReadStruct, 
                                    &cur_constVarStruct,
                                    &validRangeMutex, &cur_validRangeStruct,
                                    &staticObsMutex, &cur_staticObsStruct,
                                    &dynamicObsMutex, &cur_dynamicObsStruct,
                                    &targetFindingMutex, &cur_targetFindingStruct,
                                    &frontReachingMutex, &cur_frontReachingStruct,
                                    &lookAroundMutex, &cur_lookAroundStruct](){
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ( cur_suppressStruct.isObsStaticDominating || cur_suppressStruct.isObsDynamicDominating ){
                // std::cout << "Target finding task being suppressed!" << std::endl;
                continue;
            }   // Wait until the domination change to target finding


            // std::cout << "I am in the target finding task!" << std::endl;

            // Variables here        
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for constant
            float safe_endreach_ultimate_dist{0.0f};
            float safe_endreach_dist{0.0f};
            float safe_endreach_LR_dist{0.0f};
            int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
            {
                safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
                safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
                safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
                nTargetCount = cur_constVarStruct.nTargetCount;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for static obstacles avoidance
            float cur_distToMove{0.0f};
            int time_toMove = 1;
            preDist cur_preDist = {-1.0f, -1.0f};
            ReachEndState cur_reachEndState = { false, false, false, false };
            bool staticDodgeLeft = false;
            bool staticDodgeRight = false;
            bool isCloseToStaticObs = false;
            int nObsStaticCount = 0;
            double ObsStaticElapsed = 0.0f;
            auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
            auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(staticObsMutex);
                cur_distToMove = cur_staticObsStruct.cur_distToMove;
                time_toMove = cur_staticObsStruct.time_toMove;
                cur_preDist = cur_staticObsStruct.cur_preDist;
                cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
                staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
                staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
                isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
                nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
                ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
                obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
                obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }   

            // Variables for target finding
            targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f, -1.0f};
            float start_turning_angle{0.0f};
            float dist_to_reach{0.0f};
            float aimDirection_to_reach{0.0f};
            int nTargetFindingCount = 0;
            double TargetFindingElapsed = 0.0f;
            auto targetFindingStartTime = std::chrono::high_resolution_clock::now();
            auto targetFindingEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(targetFindingMutex);
                cur_targetCheckState = cur_targetFindingStruct.cur_targetCheckState;
                start_turning_angle = cur_targetFindingStruct.start_turning_angle;
                dist_to_reach = cur_targetFindingStruct.dist_to_reach;
                aimDirection_to_reach = cur_targetFindingStruct.aimDirection_to_reach;
                nTargetFindingCount = cur_targetFindingStruct.nTargetFindingCount;
                TargetFindingElapsed = cur_targetFindingStruct.TargetFindingElapsed;
                targetFindingStartTime = cur_targetFindingStruct.targetFindingStartTime;
                targetFindingEndTime = cur_targetFindingStruct.targetFindingEndTime;
            }

            // Variables for front reaching
            pathReachingState cur_pathReachingState = {false, false, -1.0f};
            int nfrontReachingCount = 0;
            double FrontReachingElapsed = 0.0f;
            auto frontReachingStartTime = std::chrono::high_resolution_clock::now();
            auto frontReachingEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(frontReachingMutex);
                cur_pathReachingState = cur_frontReachingStruct.cur_pathReachingState;
                nfrontReachingCount = cur_frontReachingStruct.nfrontReachingCount;
                FrontReachingElapsed = cur_frontReachingStruct.FrontReachingElapsed;
                frontReachingStartTime = cur_frontReachingStruct.frontReachingStartTime;
                frontReachingEndTime = cur_frontReachingStruct.frontReachingEndTime;
            }

            // Variables for looking around
            std::vector<angleFrontState> angleFrontState_vec;
            lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
            float ori_front{0.0f};
            int nlookAroundCount = 0;
            double LookAroundElapsed = 0.0f;
            auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
            auto lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(lookAroundMutex);
                angleFrontState_vec = cur_lookAroundStruct.angleFrontState_vec;
                cur_lookAroundState = cur_lookAroundStruct.cur_lookAroundState;
                ori_front = cur_lookAroundStruct.ori_front;
                nlookAroundCount = cur_lookAroundStruct.nlookAroundCount;
                LookAroundElapsed = cur_lookAroundStruct.LookAroundElapsed;
                lookAroundStartTime = cur_lookAroundStruct.lookAroundStartTime;
                lookAroundEndTime = cur_lookAroundStruct.lookAroundEndTime;
            }

            /*
                Target finding
                - Check that whether there has green ball or chpad as target
                - If green ball exist, check that the crazyflie has a clear path to it
                - Use the dist/aimDirection from last section
                - Can be tuned:
                -- safe dist.
                -- turning angle
                -- delay time
                -- move distance
            */ 
            // If Interrupted
            if ( has_InterruptNeedToReDo || has_InterruptNeedToReDo_dynamic ){
                if ( cur_targetCheckState.pointToTarget == false && ( cur_targetCheckState.turnStarted || cur_targetCheckState.aimTurnStarted ) ){
                    std::cout <<" Possible interruption needed redo without target..." << std::endl;
                    cur_targetCheckState.aimTurnStarted = false;
                    cur_targetCheckState.turnStarted = false;
                    on_TURNING_MODE = false;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingStartTime)
                    );
                    std::chrono::duration<double> elapsed;
                    if ( has_InterruptNeedToReDo && has_InterruptNeedToReDo_dynamic ){
                        std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                        std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                        if ( e1.count() > e2.count() ){
                            elapsed = e1;
                        }
                        else{
                            elapsed = e2;
                        }
                    }
                    else if ( has_InterruptNeedToReDo )
                        elapsed = obsStaticStartTime - targetFindingStartTime;
                    else if ( has_InterruptNeedToReDo_dynamic )
                        elapsed = obsDynamicStartTime - targetFindingStartTime;
                    if ( has_InterruptNeedToReDo )
                        has_InterruptNeedToReDo = false;
                    if ( has_InterruptNeedToReDo_dynamic )
                        has_InterruptNeedToReDo_dynamic = false;

                    std::cout <<" Original target finding start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" Interrupted after: " << elapsed.count() << "seconds(s)" << std::endl;
                    TargetFindingElapsed += elapsed.count();
                    //  targetFindingStartTime = std::chrono::high_resolution_clock::now();
                    {
                        std::lock_guard<std::mutex> lck(targetFindingMutex);
                        cur_targetFindingStruct.cur_targetCheckState.aimTurnStarted = cur_targetCheckState.aimTurnStarted;
                        cur_targetFindingStruct.cur_targetCheckState.turnStarted = cur_targetCheckState.turnStarted;   
                        cur_targetFindingStruct.TargetFindingElapsed = TargetFindingElapsed;
                    }
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                    }
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                        cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                    }
                }
            }

            if ( cur_targetCheckState.turnStarted || cur_targetCheckState.aimTurnStarted ){
                // Reset flag if the lookaround action has started
                if ( cur_lookAroundState.turnStarted || cur_lookAroundState.clearPathCheckStarted || cur_lookAroundState.nTimer != 0){
                    std::cout <<" Since some targets found, reset look around action..." << std::endl;    
                    if ( cur_lookAroundState.turnStarted )        
                        cur_lookAroundState.turnStarted = false;
                    if ( cur_lookAroundState.clearPathCheckStarted )      
                        cur_lookAroundState.clearPathCheckStarted = false;
                    if ( cur_lookAroundState.nTimer != 0 )  
                        cur_lookAroundState.nTimer = 0;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
                    );
                    const std::chrono::duration<double> elapsed = targetFindingStartTime - lookAroundStartTime;
                    std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" Interrupted after: " << elapsed.count() << "seconds(s)" << std::endl;
                    LookAroundElapsed += elapsed.count();
                    //  lookAroundStartTime = std::chrono::high_resolution_clock::now();
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.cur_lookAroundState.turnStarted = cur_lookAroundState.turnStarted;
                        cur_lookAroundStruct.cur_lookAroundState.clearPathCheckStarted = cur_lookAroundState.clearPathCheckStarted;
                        cur_lookAroundStruct.cur_lookAroundState.nTimer = cur_lookAroundState.nTimer;
                        cur_lookAroundStruct.LookAroundElapsed = LookAroundElapsed;
                    }
                }

                // Reset flag if the front reaching started
                if ( cur_pathReachingState.pathReadyToGo || cur_pathReachingState.pathOnGoing ){
                    std::cout <<" Since some targets found, reset front reaching action..." << std::endl;
                    if ( cur_pathReachingState.pathReadyToGo )  
                        cur_pathReachingState.pathReadyToGo = false;
                    if ( cur_pathReachingState.pathOnGoing )  
                        cur_pathReachingState.pathOnGoing = false;
                    on_GoTO_MODE = false;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingStartTime)
                    );
                    const std::chrono::duration<double> elapsed = targetFindingStartTime - frontReachingStartTime;
                    std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" Interrupted after: " << elapsed.count() << "seconds(s)" << std::endl;
                    FrontReachingElapsed += elapsed.count();
                    //  frontReachingStartTime = std::chrono::high_resolution_clock::now();
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.cur_pathReachingState.pathReadyToGo = cur_pathReachingState.pathReadyToGo;
                        cur_frontReachingStruct.cur_pathReachingState.pathOnGoing = cur_pathReachingState.pathOnGoing;
                        cur_frontReachingStruct.FrontReachingElapsed = FrontReachingElapsed;
                    }
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                    }
                }
            }

            // Try to point the crazyflie to the target first
            // std::cout <<" Current angle: " << aimDirection_to_reach << ", current battery state: "<< cur_state_battery_state << std::endl;  
            if ( cur_targetCheckState.pointToTarget == false && ( ( aimDirection_to_reach != -4.0f && dist_to_reach * std::cos( aimDirection_to_reach ) > 20.0f ) || cur_targetCheckState.aimTurnStarted ) ){
                if ( cur_targetCheckState.aimTurnStarted == false ){
                    targetFindingStartTime = std::chrono::high_resolution_clock::now();
                    nTargetFindingCount += 1;
                    std::cout <<" Find target start turning..." << std::endl;   
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isTargetFindingDominating = true;
                    }
                    // Stop first
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false, false);

                    if ( aimDirection_to_reach > 0.0f ){    
                        float angTurn = 90.0f / 180.0f * M_PI;
                        cur_targetCheckState.ang_toTurn = angTurn;
                        cur_targetCheckState.startAngle = cur_state_yaw;    
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn + 10.0f / 180.0f * M_PI, 4); 
                    }
                    else{
                        float angTurn = -90.0f / 180.0f * M_PI;
                        cur_targetCheckState.ang_toTurn = angTurn;
                        cur_targetCheckState.startAngle = cur_state_yaw;    
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn - 10.0f / 180.0f * M_PI, 4);
                    }
                    std::cout <<" Angle to turn: " << cur_targetCheckState.ang_toTurn << std::endl;   
                    cur_targetCheckState.aimTurnStarted = true;   
                    cur_targetCheckState.oriAimDirection =  aimDirection_to_reach;
                    on_TURNING_MODE = true;
                    
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                    }
                    {
                        std::lock_guard<std::mutex> lck(targetFindingMutex);
                        cur_targetFindingStruct.targetFindingStartTime = targetFindingStartTime;
                        cur_targetFindingStruct.nTargetFindingCount = nTargetFindingCount;
                        cur_targetFindingStruct.cur_targetCheckState.ang_toTurn = cur_targetCheckState.ang_toTurn;
                        cur_targetFindingStruct.cur_targetCheckState.startAngle = cur_targetCheckState.startAngle;
                        cur_targetFindingStruct.cur_targetCheckState.aimTurnStarted = cur_targetCheckState.aimTurnStarted;
                        cur_targetFindingStruct.cur_targetCheckState.oriAimDirection = cur_targetCheckState.oriAimDirection;
                    }  
                    continue;
                }
                else if ( cur_targetCheckState.turnStarted == false ){
                    if ( std::abs( angleDifference( cur_targetCheckState.startAngle, cur_state_yaw ) ) < std::abs(cur_targetCheckState.ang_toTurn) && cur_targetCheckState.oriAimDirection * aimDirection_to_reach > 0.0f ){
                        // Do the returning if something interrupt
                        if ( has_possibleInterrupt || has_possibleInterrupt_dynamic ){
                            std::cout <<" Some targets occur, so try to turn to look around the target again..." << std::endl;
                            auto start_time_t = std::chrono::system_clock::to_time_t(
                                std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingStartTime)
                            );
                            std::chrono::duration<double> elapsed;
                            if ( has_possibleInterrupt && has_possibleInterrupt_dynamic ){
                                std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                                std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                                if ( e1.count() > e2.count() ){
                                    elapsed = e1;
                                }
                                else{
                                    elapsed = e2;
                                }
                            }
                            else if ( has_possibleInterrupt )
                                elapsed = obsStaticStartTime - targetFindingStartTime;
                            else if ( has_possibleInterrupt_dynamic )
                                elapsed = obsDynamicStartTime - targetFindingStartTime;
                            if ( has_possibleInterrupt )
                                has_possibleInterrupt = false;
                            if ( has_possibleInterrupt_dynamic )
                                has_possibleInterrupt_dynamic = false;

                            std::cout <<" Original target finding start time: " << std::ctime(&start_time_t) << std::endl;
                            std::cout <<" Interrupted after: " << elapsed.count() << "seconds(s)" << std::endl;
                            //  TargetFindingElapsed += elapsed.count();
                            targetFindingStartTime = std::chrono::high_resolution_clock::now(); 
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isTargetFindingDominating = true;
                            }

                            float angDev = std::abs( angleDifference( cur_targetCheckState.startAngle, cur_state_yaw ) );
                            if ( cur_targetCheckState.ang_toTurn > 0.0f ){
                                float angTurn = 90.0f / 180.0f * M_PI - angDev;
                                cur_targetCheckState.ang_toTurn = angTurn;
                                cur_targetCheckState.startAngle = cur_state_yaw;    
                                Goto(od4, 0.0f, 0.0f, 0.0f, angTurn + 10.0f / 180.0f * M_PI, 2);                             
                            }
                            else{
                                float angTurn = - ( 90.0f / 180.0f * M_PI - angDev );
                                cur_targetCheckState.ang_toTurn = angTurn;
                                cur_targetCheckState.startAngle = cur_state_yaw;    
                                Goto(od4, 0.0f, 0.0f, 0.0f, angTurn - 10.0f / 180.0f * M_PI, 2);          
                            } 
                            on_TURNING_MODE = true;
                            cur_targetCheckState.oriAimDirection = aimDirection_to_reach;

                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                                cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                            }
                            {
                                std::lock_guard<std::mutex> lck(targetFindingMutex);
                                cur_targetFindingStruct.targetFindingStartTime = targetFindingStartTime;
                                cur_targetFindingStruct.cur_targetCheckState.ang_toTurn = cur_targetCheckState.ang_toTurn;
                                cur_targetFindingStruct.cur_targetCheckState.startAngle = cur_targetCheckState.startAngle;
                                cur_targetFindingStruct.cur_targetCheckState.oriAimDirection = cur_targetCheckState.oriAimDirection;
                            }  
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                            }
                        }

                        // Record the angle and front                    
                        angleFrontState state;
                        state.front = front;
                        state.angle = wrap_angle(cur_state_yaw);
                        angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                        
                        state.front = rear;
                        state.angle = wrap_angle(cur_state_yaw + M_PI);
                        angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                        state.front = left;
                        state.angle = wrap_angle(cur_state_yaw + M_PI / 2.0f);
                        angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                        state.front = right;
                        state.angle = wrap_angle(cur_state_yaw - M_PI / 2.0f);
                        angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                        // Record target angle
                        // std::cout <<" Current aim direction diff:" << std::abs( aimDirection_to_reach ) << std::endl;   
                        if ( cur_targetCheckState.cur_aimDiff > std::abs( aimDirection_to_reach ) ){
                            cur_targetCheckState.targetAngle = cur_state_yaw;
                            cur_targetCheckState.cur_aimDiff = std::abs( aimDirection_to_reach );
                            std::cout <<" Current angle: "<< cur_state_yaw << " and aim diff: " << std::abs( aimDirection_to_reach ) << std::endl;   
                        }
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                        }
                        {
                            std::lock_guard<std::mutex> lck(targetFindingMutex);
                            cur_targetFindingStruct.cur_targetCheckState.targetAngle = cur_targetCheckState.targetAngle;
                            cur_targetFindingStruct.cur_targetCheckState.cur_aimDiff = cur_targetCheckState.cur_aimDiff;
                        }
                    }
                    else{
                        // Check the target angle to go to ( the clear path)
                        // Stop the turning action
                        std::cout <<" Complete turning and start to check..." << std::endl;   
                        Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
                        // Sort the angle array first           
                        std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(),
                            [cur_targetCheckState](const angleFrontState& a, const angleFrontState& b) -> bool {
                                float angledev_a = std::abs(angleDifference(a.angle, cur_targetCheckState.targetAngle));
                                float angledev_b = std::abs(angleDifference(b.angle, cur_targetCheckState.targetAngle));
                                if (angledev_a == angledev_b) {
                                    return a.angle < b.angle;
                                }
                                return angledev_a < angledev_b;
                            }
                        );

                        // Check for clear path
                        bool hasObOnPath = false; 
                        for ( const auto& pair_cand : angleFrontState_vec ){
                            if ( pair_cand.front <= safe_endreach_dist + 0.7 / 2){
                                continue;
                            }

                            float angMin = std::abs( std::atan2( 0.1f, pair_cand.front ) );
                            hasObOnPath = false;
                            for ( const auto& pair_to_compare : angleFrontState_vec ){
                                float angDev = std::abs( angleDifference( pair_to_compare.angle, pair_cand.angle ) );
                                if ( angDev <= angMin ){
                                    if ( pair_to_compare.front * std::cos( angDev ) < pair_cand.front - safe_endreach_dist ){
                                        hasObOnPath = true;
                                        break;
                                    }
                                }
                                else if ( angDev <= 45.0f / 180.0f * M_PI && angDev > angMin ){
                                    if ( pair_to_compare.front * std::sin( angDev ) <= 0.1f ){
                                        hasObOnPath = true;
                                        break;
                                    }
                                }                        
                            }

                            if ( hasObOnPath == false ){
                                // Found the path, turn to that angle
                                cur_targetCheckState.targetAngle = pair_cand.angle;

                                // Try to turn to the angle
                                std::cout << "Ready to turn to the target found angle..." << std::endl;
                                //  std::cout <<" Current angle to turn: " << cur_state_yaw <<", Target angle: " << pair_cand.angle << std::endl; 
                                //  std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_targetCheckState.targetAngle ) << std::endl; 
                                float angTurn = angleDifference( cur_state_yaw, pair_cand.angle ) + 5.0f / 180.0f * M_PI;
                                if ( angleDifference( cur_state_yaw, pair_cand.angle ) < 0.0f ){
                                    angTurn -= 10.0f / 180.0f * M_PI;
                                }
                                Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1);
                                cur_targetCheckState.turnStarted = true;
                                break;
                            }
                        }
                    }

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(targetFindingMutex);
                        cur_targetFindingStruct.cur_targetCheckState.targetAngle = cur_targetCheckState.targetAngle;
                        cur_targetFindingStruct.cur_targetCheckState.turnStarted = cur_targetCheckState.turnStarted;
                    }  
                    continue;
                }
                else{
                    std::atomic<float> yaw{0.0f};
                    {
                        yaw.store(cur_state_yaw, std::memory_order_relaxed);
                    }

                    if ( std::abs( angleDifference( cur_targetCheckState.targetAngle, yaw ) ) >= 5.0f / 180.0f * M_PI ){
                        // Do the returning if something interrupt
                        if ( has_possibleInterrupt|| has_possibleInterrupt_dynamic ){
                            std::cout <<" Some targets occur, so try to turn to the target again..." << std::endl;
                            auto start_time_t = std::chrono::system_clock::to_time_t(
                                std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingStartTime)
                            );
                            std::chrono::duration<double> elapsed;
                            if ( has_possibleInterrupt && has_possibleInterrupt_dynamic ){
                                std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                                std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                                if ( e1.count() > e2.count() ){
                                    elapsed = e1;
                                }
                                else{
                                    elapsed = e2;
                                }
                            }
                            else if ( has_possibleInterrupt )
                                elapsed = obsStaticStartTime - targetFindingStartTime;
                            else if ( has_possibleInterrupt_dynamic )
                                elapsed = obsDynamicStartTime - targetFindingStartTime;
                            if ( has_possibleInterrupt )
                                has_possibleInterrupt = false;
                            if ( has_possibleInterrupt_dynamic )
                                has_possibleInterrupt_dynamic = false;

                            std::cout <<" Original target finding start time: " << std::ctime(&start_time_t) << std::endl;
                            std::cout <<" Interrupted after: " << elapsed.count() << "seconds(s)" << std::endl;
                            //  TargetFindingElapsed += elapsed.count();
                            targetFindingStartTime = std::chrono::high_resolution_clock::now();
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isTargetFindingDominating = true;
                            }
                            
                            float angTurn = angleDifference( yaw, cur_targetCheckState.targetAngle ) + 5.0f / 180.0f * M_PI;
                            if ( angleDifference( yaw, cur_targetCheckState.targetAngle ) < 0.0f ){
                                angTurn -= 10.0f / 180.0f * M_PI;
                            }
                            Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1); 
                            on_TURNING_MODE = true; 
                            
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                                cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                                cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                            }  
                            {
                                std::lock_guard<std::mutex> lck(targetFindingMutex);
                                cur_targetFindingStruct.targetFindingStartTime = targetFindingStartTime;
                            }  
                            {
                                std::lock_guard<std::mutex> lck(validRangeMutex);
                                cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                            }
                        }                        
                        // continue turning
                        //  std::cout <<" Keep turning with angle diff: " << angleDifference( cur_state_yaw, cur_targetCheckState.targetAngle ) << ", with current angle: " << cur_state_yaw << std::endl;  
                        continue;
                    }
                    else{ 
                        // Stop first
                        Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));

                        // If current front is not goable
                        bool hasNotFoundPath = false;
                        if ( front <= safe_endreach_dist + 0.7 / 2 ){
                            std::cout <<" Turn to the target angle, But current angle is not goable..." << std::endl;
                            angleFrontState state;
                            state.front = front;
                            state.angle = wrap_angle(yaw);
                            angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                            {
                                std::lock_guard<std::mutex> lck(lookAroundMutex);
                                cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                            }   

                            // Start to find another way to go to
                            // Check for clear path
                            bool hasObOnPath = false; 
                            for ( const auto& pair_cand : angleFrontState_vec ){
                                if ( pair_cand.front <= safe_endreach_dist + 0.7 / 2 ){
                                    continue;
                                }

                                if ( std::abs( angleDifference( yaw, pair_cand.angle ) ) <= 7.5 / 180.0f * M_PI ){
                                    continue;
                                }

                                float angMin = std::abs( std::atan2( 0.1f, pair_cand.front ) );
                                hasObOnPath = false;
                                for ( const auto& pair_to_compare : angleFrontState_vec ){
                                    float angDev = std::abs( angleDifference( pair_to_compare.angle, pair_cand.angle ) );
                                    if ( angDev <= angMin ){
                                        if ( pair_to_compare.front * std::cos( angDev ) < pair_cand.front - safe_endreach_dist ){
                                            hasObOnPath = true;
                                            break;
                                        }
                                    }
                                    else if ( angDev <= 45.0f / 180.0f * M_PI && angDev > angMin ){
                                        if ( pair_to_compare.front * std::sin( angDev ) <= 0.1f ){
                                            hasObOnPath = true;
                                            break;
                                        }
                                    }                        
                                }
            
                                if ( hasObOnPath == false ){
                                    // Found the path, turn to that angle
                                    cur_targetCheckState.targetAngle = pair_cand.angle;
            
                                    // Try to turn to the angle
                                    std::cout << "Ready to turn to the target found angle..." << std::endl;
                                    //  std::cout <<" Current angle: " << cur_state_yaw <<", Target angle: " << pair_cand.angle << std::endl; 
                                    //  std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_targetCheckState.targetAngle ) << std::endl; 
                                    float angTurn = angleDifference( yaw, pair_cand.angle ) + 5.0f / 180.0f * M_PI;
                                    if ( angleDifference( yaw, pair_cand.angle ) < 0.0f ){
                                        angTurn -= 10.0f / 180.0f * M_PI;
                                    }
                                    Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 2);
                                    break;
                                }
                            }
                            if ( hasObOnPath == false ){
                                // Set variables back
                                {
                                    std::lock_guard<std::mutex> lck(targetFindingMutex);
                                    cur_targetFindingStruct.cur_targetCheckState.targetAngle = cur_targetCheckState.targetAngle;
                                }   
                                continue;      
                            }        
                            else{
                                hasNotFoundPath = true;
                            }          
                        }

                        // Ready to go to path
                        if ( hasNotFoundPath == false ){
                            std::cout <<" Turn to the target angle, ready to go to it..." << std::endl;
                            cur_pathReachingState.pathReadyToGo = true;
                            cur_targetCheckState.pointToTarget = true;
                            ori_front = front;
                            {
                                std::lock_guard<std::mutex> lck(frontReachingMutex);
                                cur_frontReachingStruct.cur_pathReachingState.pathReadyToGo = cur_pathReachingState.pathReadyToGo;
                            }  
                            {
                                std::lock_guard<std::mutex> lck(targetFindingMutex);
                                cur_targetFindingStruct.cur_targetCheckState.pointToTarget = cur_targetCheckState.pointToTarget;
                            }  
                            {
                                std::lock_guard<std::mutex> lck(lookAroundMutex);
                                cur_lookAroundStruct.ori_front = cur_lookAroundStruct.ori_front;
                            } 
                         
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isTargetFindingDominating = false;
                                cur_suppressStruct.isFrontReachingDominating= true;
                            }
                        }
                        else{
                            std::cout <<" Can not find goable direction, restart look around..." << std::endl;
                            {
                                std::lock_guard<std::mutex> lck(suppressMutex);
                                cur_suppressStruct.isTargetFindingDominating = false;
                            }
                        }

                        // Reset other flags
                        cur_targetCheckState.aimTurnStarted = false;
                        cur_targetCheckState.turnStarted = false;
                        on_TURNING_MODE = false;

                        targetFindingEndTime = std::chrono::high_resolution_clock::now();
                        const std::chrono::duration<double> elapsed = targetFindingEndTime - targetFindingStartTime;

                        auto start_time_t = std::chrono::system_clock::to_time_t(
                            std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingStartTime)
                        );
                        auto end_time_t = std::chrono::system_clock::to_time_t(
                            std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingEndTime)
                        );

                        std::cout <<" Target finding complete with start time: " << std::ctime(&start_time_t) << std::endl;
                        std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                        std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                        TargetFindingElapsed += elapsed.count();
                        std::cout <<" , average target finding elapsed: " << TargetFindingElapsed / nTargetFindingCount << " seconds(s)" << std::endl;
                        std::cout <<" , so far has done target finding for " << nTargetFindingCount << " times" << std::endl;
                        
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(targetFindingMutex);
                            cur_targetFindingStruct.cur_targetCheckState.aimTurnStarted = cur_targetCheckState.aimTurnStarted;
                            cur_targetFindingStruct.cur_targetCheckState.turnStarted = cur_targetCheckState.turnStarted;
                            cur_targetFindingStruct.targetFindingEndTime = targetFindingEndTime;
                            cur_targetFindingStruct.TargetFindingElapsed = TargetFindingElapsed;
                        }  

                        {
                            std::lock_guard<std::mutex> lck(validRangeMutex);
                            cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                        }
                    }    
                    continue;
                }
            }
        }        
    });

    std::thread FrontReachingTask([od4, &isTerminateThread,
                                    &suppressMutex, &cur_suppressStruct,
                                    &cur_sensorReadStruct, 
                                    &cur_constVarStruct,
                                    &validRangeMutex, &cur_validRangeStruct,
                                    &staticObsMutex, &cur_staticObsStruct,
                                    &dynamicObsMutex, &cur_dynamicObsStruct,
                                    &targetFindingMutex, &cur_targetFindingStruct,
                                    &frontReachingMutex, &cur_frontReachingStruct,
                                    &lookAroundMutex, &cur_lookAroundStruct](){
        while( od4->isRunning() && isTerminateThread == false ){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ( cur_suppressStruct.isObsStaticDominating || cur_suppressStruct.isObsDynamicDominating
                 || cur_suppressStruct.isTargetFindingDominating || cur_suppressStruct.isSupressFrontReaching ){
                // std::cout << "Front reaching task being suppressed!" << std::endl;
                continue;
            }   // Wait until the domination change to target finding

            // std::cout << "I am in the front reaching task!" << std::endl;

            // Variables here        
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for constant
            float safe_endreach_ultimate_dist{0.0f};
            float safe_endreach_dist{0.0f};
            float safe_endreach_LR_dist{0.0f};
            int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
            {
                safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
                safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
                safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
                nTargetCount = cur_constVarStruct.nTargetCount;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for static obstacles avoidance
            float cur_distToMove{0.0f};
            int time_toMove = 1;
            preDist cur_preDist = {-1.0f, -1.0f};
            ReachEndState cur_reachEndState = { false, false, false, false };
            bool staticDodgeLeft = false;
            bool staticDodgeRight = false;
            bool isCloseToStaticObs = false;
            int nObsStaticCount = 0;
            double ObsStaticElapsed = 0.0f;
            auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
            auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(staticObsMutex);
                cur_distToMove = cur_staticObsStruct.cur_distToMove;
                time_toMove = cur_staticObsStruct.time_toMove;
                cur_preDist = cur_staticObsStruct.cur_preDist;
                cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
                staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
                staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
                isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
                nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
                ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
                obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
                obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }   

            // Variables for target finding
            targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f, -1.0f};
            float start_turning_angle{0.0f};
            float dist_to_reach{0.0f};
            float aimDirection_to_reach{0.0f};
            int nTargetFindingCount = 0;
            double TargetFindingElapsed = 0.0f;
            auto targetFindingStartTime = std::chrono::high_resolution_clock::now();
            auto targetFindingEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(targetFindingMutex);
                cur_targetCheckState = cur_targetFindingStruct.cur_targetCheckState;
                start_turning_angle = cur_targetFindingStruct.start_turning_angle;
                dist_to_reach = cur_targetFindingStruct.dist_to_reach;
                aimDirection_to_reach = cur_targetFindingStruct.aimDirection_to_reach;
                nTargetFindingCount = cur_targetFindingStruct.nTargetFindingCount;
                TargetFindingElapsed = cur_targetFindingStruct.TargetFindingElapsed;
                targetFindingStartTime = cur_targetFindingStruct.targetFindingStartTime;
                targetFindingEndTime = cur_targetFindingStruct.targetFindingEndTime;
            }

            // Variables for front reaching
            pathReachingState cur_pathReachingState = {false, false, -1.0f};
            int nfrontReachingCount = 0;
            double FrontReachingElapsed = 0.0f;
            auto frontReachingStartTime = std::chrono::high_resolution_clock::now();
            auto frontReachingEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(frontReachingMutex);
                cur_pathReachingState = cur_frontReachingStruct.cur_pathReachingState;
                nfrontReachingCount = cur_frontReachingStruct.nfrontReachingCount;
                FrontReachingElapsed = cur_frontReachingStruct.FrontReachingElapsed;
                frontReachingStartTime = cur_frontReachingStruct.frontReachingStartTime;
                frontReachingEndTime = cur_frontReachingStruct.frontReachingEndTime;
            }

            // Variables for looking around
            std::vector<angleFrontState> angleFrontState_vec;
            lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
            float ori_front{0.0f};
            int nlookAroundCount = 0;
            double LookAroundElapsed = 0.0f;
            auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
            auto lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(lookAroundMutex);
                angleFrontState_vec = cur_lookAroundStruct.angleFrontState_vec;
                cur_lookAroundState = cur_lookAroundStruct.cur_lookAroundState;
                ori_front = cur_lookAroundStruct.ori_front;
                nlookAroundCount = cur_lookAroundStruct.nlookAroundCount;
                LookAroundElapsed = cur_lookAroundStruct.LookAroundElapsed;
                lookAroundStartTime = cur_lookAroundStruct.lookAroundStartTime;
                lookAroundEndTime = cur_lookAroundStruct.lookAroundEndTime;
            }
            
            /*
                Front reaching
                - Go to the path straightforwardly if permitted
            */
            // If meets the front/left/right end, reset all flags
            if ( cur_reachEndState.reachFront == true || cur_reachEndState.reachLeft == true || cur_reachEndState.reachRight == true ){
                if ( cur_pathReachingState.pathReadyToGo || cur_pathReachingState.pathOnGoing ){
                    std::cout <<" Reach end stop going with front: " << cur_reachEndState.reachFront << ", left: " << cur_reachEndState.reachLeft << ", right: " << cur_reachEndState.reachRight << std::endl;
                    
                    cur_pathReachingState.pathReadyToGo = false;
                    cur_pathReachingState.pathOnGoing = false;
                    
                    cur_targetCheckState.pointToTarget = false;
                    cur_targetCheckState.cur_aimDiff = 100.0f / 180.0f * M_PI;
        
                    // Initialize the vector here
                    if ( angleFrontState_vec.size() > 0 ){
                        angleFrontState_vec.clear();
                    }

                    on_GoTO_MODE = false;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingStartTime)
                    );
                    const std::chrono::duration<double> elapsed = obsStaticStartTime - frontReachingStartTime;
                    std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" Interrupted (due to reach ends) after: " << elapsed.count() << "seconds(s)" << std::endl;
                    FrontReachingElapsed += elapsed.count();
                    //  frontReachingStartTime = std::chrono::high_resolution_clock::now();
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.cur_pathReachingState.pathReadyToGo = cur_pathReachingState.pathReadyToGo;
                        cur_frontReachingStruct.cur_pathReachingState.pathOnGoing = cur_pathReachingState.pathOnGoing;
                        cur_frontReachingStruct.FrontReachingElapsed = FrontReachingElapsed;
                    }
                    {
                        std::lock_guard<std::mutex> lck(targetFindingMutex);
                        cur_targetFindingStruct.cur_targetCheckState.pointToTarget = cur_targetCheckState.pointToTarget;
                        cur_targetFindingStruct.cur_targetCheckState.cur_aimDiff = cur_targetCheckState.cur_aimDiff;
                    }
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                    }
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                    }
                }
            }

            // If being interrupted, try to go to the original path again
            if ( has_possibleInterrupt || has_possibleInterrupt_dynamic || has_InterruptNeedToReDo || has_InterruptNeedToReDo_dynamic ){
                if ( cur_pathReachingState.pathReadyToGo && cur_pathReachingState.pathOnGoing ){
                    std::cout <<" Being interrupted and try to go again..." << std::endl;
                    cur_pathReachingState.pathOnGoing = false;
                    std::chrono::duration<double> elapsed;
                    if ( ( has_possibleInterrupt && has_possibleInterrupt_dynamic ) || ( has_InterruptNeedToReDo && has_InterruptNeedToReDo_dynamic ) ){
                        std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                        std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                        if ( e1.count() > e2.count() ){
                            elapsed = e1;
                        }
                        else{
                            elapsed = e2;
                        }
                    }
                    else if ( has_possibleInterrupt || has_InterruptNeedToReDo )
                        elapsed = obsStaticStartTime - targetFindingStartTime;
                    else if ( has_possibleInterrupt_dynamic || has_InterruptNeedToReDo_dynamic )
                        elapsed = obsDynamicStartTime - targetFindingStartTime;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingStartTime)
                    );
                    //  const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - frontReachingStartTime;
                    std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" Interrupted time(due to dodging) is: " << elapsed.count() << "seconds(s)" << std::endl;
                    FrontReachingElapsed += elapsed.count();    

                    if ( has_possibleInterrupt )
                        has_possibleInterrupt = false;
                    if ( has_possibleInterrupt_dynamic )
                        has_possibleInterrupt_dynamic = false;
                    if ( has_InterruptNeedToReDo )
                        has_InterruptNeedToReDo = false;
                    if ( has_InterruptNeedToReDo_dynamic )
                        has_InterruptNeedToReDo_dynamic = false;

                    // Restart the timer again             
                    frontReachingStartTime = std::chrono::high_resolution_clock::now(); 

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.cur_pathReachingState.pathOnGoing = cur_pathReachingState.pathOnGoing;
                        cur_frontReachingStruct.FrontReachingElapsed = FrontReachingElapsed;
                        cur_frontReachingStruct.frontReachingStartTime = frontReachingStartTime;
                    }
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                        cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                        cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                        cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                    }
                }
            }

            // Go to path
            if ( cur_pathReachingState.pathReadyToGo ){
                if ( cur_pathReachingState.pathOnGoing == false ){
                    frontReachingStartTime = std::chrono::high_resolution_clock::now();
                    nfrontReachingCount += 1;
                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isFrontReachingDominating = true;
                    }
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                    std::cout <<" Start go to action with front: " << front << std::endl;
                    cur_distToMove = front - safe_endreach_dist / 2.0f;
                    if ( cur_distToMove >= 1.0f )
                        time_toMove = 4;
                    else
                        time_toMove = 5;
                    if ( cur_targetCheckState.pointToTarget ){
                        cur_distToMove = 0.7f;
                        time_toMove = 2;
                        if ( front < 0.7f){
                            cur_distToMove = front - safe_endreach_dist / 2.0f;
                            time_toMove = 5;
                        }
                    }
                    Goto(od4, cur_distToMove * std::cos( cur_state_yaw ), cur_distToMove * std::sin( cur_state_yaw ), 0.0f, 0.0f, time_toMove);
                    cur_pathReachingState.pathOnGoing = true;
                    cur_pathReachingState.startFront = front;
                    on_GoTO_MODE = true;

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.frontReachingStartTime = frontReachingStartTime;
                        cur_frontReachingStruct.nfrontReachingCount = nfrontReachingCount;
                        cur_frontReachingStruct.cur_pathReachingState.pathOnGoing = cur_pathReachingState.pathOnGoing;
                        cur_frontReachingStruct.cur_pathReachingState.startFront = cur_pathReachingState.startFront;
                    }
                    {
                        std::lock_guard<std::mutex> lck(staticObsMutex);
                        cur_staticObsStruct.cur_distToMove = cur_distToMove;
                        cur_staticObsStruct.time_toMove = time_toMove;
                    }
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                    }
                }
                else{
                    if ( dist_to_reach > -1.0f ){
                        // Reach the target, stop current action
                        if ( dist_to_reach <= 80.0f ){
                            std::cout <<" Reach target with target exist..." << std::endl;
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);

                            // Reset flags
                            on_GoTO_MODE = false;
                        }
                        else if ( std::abs( cur_pathReachingState.startFront - front ) >= 0.6f && front < cur_pathReachingState.startFront ){
                            std::cout <<" Reach target with 0.6 range of current front: " << front << std::endl;
                            Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);

                            // Reset flags
                            on_GoTO_MODE = false;
                        }
                    }
                    else if ( std::abs( cur_pathReachingState.startFront - front ) >= cur_distToMove - safe_endreach_dist && front < cur_pathReachingState.startFront ){
                        // Reach the target, stop current action
                        std::cout <<" Reach target with front distance reached, pre front: " << cur_pathReachingState.startFront << ", cur front: " << front << std::endl;
                        Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);

                        // Reset flags
                        on_GoTO_MODE = false;
                    }
                }

                if ( on_GoTO_MODE ){
                    continue;
                }
                else{     
                    // Reset while the goto mode being set to false
                    cur_pathReachingState.pathOnGoing = false;
                    cur_pathReachingState.pathReadyToGo = false;
                    
                    cur_targetCheckState.pointToTarget = false;
                    cur_targetCheckState.cur_aimDiff = 100.0f / 180.0f * M_PI;
        
                    // Initialize the vector here
                    if ( angleFrontState_vec.size() > 0 ){
                        angleFrontState_vec.clear();
                    }

                    {
                        std::lock_guard<std::mutex> lck(suppressMutex);
                        cur_suppressStruct.isFrontReachingDominating = false;
                    }

                    frontReachingEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = frontReachingEndTime - frontReachingStartTime;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingEndTime)
                    );

                    std::cout <<" Front reaching complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    FrontReachingElapsed += elapsed.count();
                    std::cout <<" , average front reaching elapsed: " << FrontReachingElapsed / nfrontReachingCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has done front reaching for " << nfrontReachingCount << " times" << std::endl;
                    
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_GoTO_MODE = on_GoTO_MODE;
                    }
                    {
                        std::lock_guard<std::mutex> lck(targetFindingMutex);
                        cur_targetFindingStruct.cur_targetCheckState.pointToTarget = cur_targetCheckState.pointToTarget;
                        cur_targetFindingStruct.cur_targetCheckState.cur_aimDiff = cur_targetCheckState.cur_aimDiff;
                    }  
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.cur_pathReachingState.pathOnGoing = cur_pathReachingState.pathOnGoing;
                        cur_frontReachingStruct.cur_pathReachingState.pathReadyToGo = cur_pathReachingState.pathReadyToGo;
                        cur_frontReachingStruct.frontReachingEndTime = frontReachingEndTime;
                        cur_frontReachingStruct.FrontReachingElapsed = FrontReachingElapsed;
                    }
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                    }   
                }
            }
        }        
    });

    std::thread LookAroundTask([od4, &isTerminateThread,
                                &suppressMutex, &cur_suppressStruct,
                                &cur_sensorReadStruct, 
                                &cur_constVarStruct,
                                &validRangeMutex, &cur_validRangeStruct,
                                &staticObsMutex, &cur_staticObsStruct,
                                &dynamicObsMutex, &cur_dynamicObsStruct,
                                &targetFindingMutex, &cur_targetFindingStruct,
                                &frontReachingMutex, &cur_frontReachingStruct,
                                &lookAroundMutex, &cur_lookAroundStruct](){
        while( od4->isRunning() && isTerminateThread == false ){            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ( cur_suppressStruct.isObsStaticDominating || cur_suppressStruct.isObsDynamicDominating
                || cur_suppressStruct.isTargetFindingDominating || cur_suppressStruct.isFrontReachingDominating ){
                // std::cout << "look around task being suppressed!" << std::endl;
                continue;
            }   // Wait until the domination change to target finding

            // std::cout << "I am in the look around task!" << std::endl;

            // Variables here        
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            float cur_state_yaw{0.0f};
            float cur_state_battery_state{0.0f};
            float dist_target{-1.0f};
            float dist_obs{-1.0f};
            float dist_chpad{-1.0f};
            float aimDirection_target{-4.0f};
            float aimDirection_obs{-4.0f};
            float aimDirection_chpad{-4.0f};
            float closeBallTimer{0.0f};
            int16_t closeBallCount{0}; 
            float closeStaticObsTimer{0.0f};
            int16_t closeStaticObsCount{0}; 
            int16_t nTargetTimer{0};
            int16_t is_chpad_found{0}; 
            {
                front = cur_sensorReadStruct.front;
                rear = cur_sensorReadStruct.rear;
                left = cur_sensorReadStruct.left;
                right = cur_sensorReadStruct.right;
                cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
                cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
                dist_target = cur_sensorReadStruct.dist_target;
                dist_obs = cur_sensorReadStruct.dist_obs;
                dist_chpad = cur_sensorReadStruct.dist_chpad;
                aimDirection_target = cur_sensorReadStruct.aimDirection_target;
                aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
                aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
                closeBallTimer = cur_sensorReadStruct.closeBallTimer;
                closeBallCount = cur_sensorReadStruct.closeBallCount;
                closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
                closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
                nTargetTimer = cur_sensorReadStruct.nTargetTimer;
                is_chpad_found = cur_sensorReadStruct.is_chpad_found;
            }

            // Variables for constant
            float safe_endreach_ultimate_dist{0.0f};
            float safe_endreach_dist{0.0f};
            float safe_endreach_LR_dist{0.0f};
            int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
            {
                safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
                safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
                safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
                nTargetCount = cur_constVarStruct.nTargetCount;
            }

            // Variables for valid range
            std::vector<distPathState> distPathstate_vec;
            ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
            bool on_GoTO_MODE = false;
            bool on_TURNING_MODE = false;
            float preFront_togo{-1.0f};
            {
                std::lock_guard<std::mutex> lck(validRangeMutex);
                distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
                cur_validWay = cur_validRangeStruct.cur_validWay;
                on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
                on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
                preFront_togo = cur_validRangeStruct.preFront_togo;
            }

            // Variables for static obstacles avoidance
            float cur_distToMove{0.0f};
            int time_toMove = 1;
            preDist cur_preDist = {-1.0f, -1.0f};
            ReachEndState cur_reachEndState = { false, false, false, false };
            bool staticDodgeLeft = false;
            bool staticDodgeRight = false;
            bool isCloseToStaticObs = false;
            int nObsStaticCount = 0;
            double ObsStaticElapsed = 0.0f;
            auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
            auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(staticObsMutex);
                cur_distToMove = cur_staticObsStruct.cur_distToMove;
                time_toMove = cur_staticObsStruct.time_toMove;
                cur_preDist = cur_staticObsStruct.cur_preDist;
                cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
                staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
                staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
                isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
                nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
                ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
                obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
                obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
            } 

            float dodgeDist{0.0f};
            float dodgeDist_UP{0.0f};
            bool has_dodgeToRear = false;
            DodgeType cur_dodgeType = DODGE_NONE;
            obsState cur_obsState = { -1.0f, -1.0f };
            bool has_possibleInterrupt = false;
            bool has_possibleInterrupt_dynamic = false;
            bool has_InterruptNeedToReDo = false;
            bool has_InterruptNeedToReDo_dynamic = false;
            int nObsDynamicCount = 0;
            double ObsDynamicElapsed = 0.0f;
            auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
            auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(dynamicObsMutex);
                dodgeDist = cur_dynamicObsStruct.dodgeDist;
                dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
                has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
                cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
                cur_obsState = cur_dynamicObsStruct.cur_obsState;
                has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
                has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
                has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
                has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
                nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
                ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
                obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
                obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
            }   

            // Variables for target finding
            targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f, -1.0f};
            float start_turning_angle{0.0f};
            float dist_to_reach{0.0f};
            float aimDirection_to_reach{0.0f};
            int nTargetFindingCount = 0;
            double TargetFindingElapsed = 0.0f;
            auto targetFindingStartTime = std::chrono::high_resolution_clock::now();
            auto targetFindingEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(targetFindingMutex);
                cur_targetCheckState = cur_targetFindingStruct.cur_targetCheckState;
                start_turning_angle = cur_targetFindingStruct.start_turning_angle;
                dist_to_reach = cur_targetFindingStruct.dist_to_reach;
                aimDirection_to_reach = cur_targetFindingStruct.aimDirection_to_reach;
                nTargetFindingCount = cur_targetFindingStruct.nTargetFindingCount;
                TargetFindingElapsed = cur_targetFindingStruct.TargetFindingElapsed;
                targetFindingStartTime = cur_targetFindingStruct.targetFindingStartTime;
                targetFindingEndTime = cur_targetFindingStruct.targetFindingEndTime;
            }

            // Variables for front reaching
            pathReachingState cur_pathReachingState = {false, false, -1.0f};
            int nfrontReachingCount = 0;
            double FrontReachingElapsed = 0.0f;
            auto frontReachingStartTime = std::chrono::high_resolution_clock::now();
            auto frontReachingEndTime = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lck(frontReachingMutex);
                cur_pathReachingState = cur_frontReachingStruct.cur_pathReachingState;
                nfrontReachingCount = cur_frontReachingStruct.nfrontReachingCount;
                FrontReachingElapsed = cur_frontReachingStruct.FrontReachingElapsed;
                frontReachingStartTime = cur_frontReachingStruct.frontReachingStartTime;
                frontReachingEndTime = cur_frontReachingStruct.frontReachingEndTime;
            }

            // Variables for looking around
            std::vector<angleFrontState> angleFrontState_vec;
            lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
            float ori_front{0.0f};
            int nlookAroundCount = 0;
            double LookAroundElapsed = 0.0f;
            auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
            auto lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
            {
                std::lock_guard<std::mutex> lck(lookAroundMutex);
                angleFrontState_vec = cur_lookAroundStruct.angleFrontState_vec;
                cur_lookAroundState = cur_lookAroundStruct.cur_lookAroundState;
                ori_front = cur_lookAroundStruct.ori_front;
                nlookAroundCount = cur_lookAroundStruct.nlookAroundCount;
                LookAroundElapsed = cur_lookAroundStruct.LookAroundElapsed;
                lookAroundStartTime = cur_lookAroundStruct.lookAroundStartTime;
                lookAroundEndTime = cur_lookAroundStruct.lookAroundEndTime;
            }

            /*
                Look around
                - Turn around for 360 degree to see whether target exist
                - In the mean time, record some possible front path to go to
            */
            // If Interrupted
            if ( has_InterruptNeedToReDo || has_InterruptNeedToReDo_dynamic ){
                if ( cur_lookAroundState.turnStarted || cur_lookAroundState.clearPathCheckStarted ){
                    std::cout <<" Possible interruption to reset look around..." << std::endl;
                    cur_lookAroundState.turnStarted = false;
                    cur_lookAroundState.clearPathCheckStarted = false;
                    cur_lookAroundState.nTimer = 0;
                    on_TURNING_MODE = false;

                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.cur_lookAroundState.turnStarted = cur_lookAroundState.turnStarted;
                        cur_lookAroundStruct.cur_lookAroundState.clearPathCheckStarted = cur_lookAroundState.clearPathCheckStarted;
                        cur_lookAroundStruct.cur_lookAroundState.nTimer = cur_lookAroundState.nTimer;
                    }   
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                    }
                }

                std::chrono::duration<double> elapsed;
                if ( has_InterruptNeedToReDo && has_InterruptNeedToReDo_dynamic ){
                    std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                    std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                    if ( e1.count() > e2.count() ){
                        elapsed = e1;
                    }
                    else{
                        elapsed = e2;
                    }
                }
                else if ( has_InterruptNeedToReDo )
                    elapsed = obsStaticStartTime - targetFindingStartTime;
                else if ( has_InterruptNeedToReDo_dynamic )
                    elapsed = obsDynamicStartTime - targetFindingStartTime;

                auto start_time_t = std::chrono::system_clock::to_time_t(
                    std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
                );
                //  const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - lookAroundStartTime;
                std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
                std::cout <<" Interrupted time(due to dodging) after: " << elapsed.count() << "seconds(s)" << std::endl;
                LookAroundElapsed += elapsed.count();
                //  lookAroundStartTime = std::chrono::high_resolution_clock::now();
                if ( has_InterruptNeedToReDo )
                    has_InterruptNeedToReDo = false;
                else if ( has_InterruptNeedToReDo_dynamic )
                    has_InterruptNeedToReDo_dynamic = false;             

                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(lookAroundMutex);
                    cur_lookAroundStruct.LookAroundElapsed = LookAroundElapsed;
                }   
                {
                    std::lock_guard<std::mutex> lck(dynamicObsMutex);
                    cur_dynamicObsStruct.has_InterruptNeedToReDo = has_InterruptNeedToReDo;
                    cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic = has_InterruptNeedToReDo_dynamic;
                }
            }

            // Try to find a clear path close to the target
            // std::cout <<" Run to here with clearPathCheckStarted: " << cur_lookAroundState.clearPathCheckStarted << std::endl;
            if ( cur_lookAroundState.clearPathCheckStarted == false ){
                std::cout <<" No target exist, ready to turn around to find" << std::endl;
                if ( cur_lookAroundState.nTimer == 0 ){
                    nlookAroundCount += 1;
                    lookAroundStartTime = std::chrono::high_resolution_clock::now();
                    if ( angleFrontState_vec.size() > 0 ){
                        // Initialize the vector here
                        angleFrontState_vec.clear();
                    }
                    cur_lookAroundState.preAngle = cur_state_yaw + M_PI;
                }
                cur_lookAroundState.startAngle = cur_state_yaw;
                Goto(od4, 0.0f, 0.0f, 0.0f, 120.0f / 180.0f * M_PI, 2);
                cur_lookAroundState.clearPathCheckStarted = true;
                on_TURNING_MODE = true;             

                // Set variables back
                {
                    std::lock_guard<std::mutex> lck(lookAroundMutex);
                    cur_lookAroundStruct.nlookAroundCount = nlookAroundCount;
                    cur_lookAroundStruct.lookAroundStartTime = lookAroundStartTime;
                    cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                    cur_lookAroundStruct.cur_lookAroundState.preAngle = cur_lookAroundState.preAngle;
                    cur_lookAroundStruct.cur_lookAroundState.startAngle = cur_lookAroundState.startAngle;
                    cur_lookAroundStruct.cur_lookAroundState.clearPathCheckStarted = cur_lookAroundState.clearPathCheckStarted;
                }   
                {
                    std::lock_guard<std::mutex> lck(validRangeMutex);
                    cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                }
            }
            else if ( cur_lookAroundState.turnStarted == false ){
                if ( std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state_yaw ) ) < 110.0f / 180.0f * M_PI ){
                    if ( has_possibleInterrupt || has_possibleInterrupt_dynamic ){
                        std::cout <<" Some targets occur, so try to look up again..." << std::endl;
                        float angTurn = 120.0f / 180.0f * M_PI - std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state_yaw ) );
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 2);
                        on_TURNING_MODE = true;

                        std::chrono::duration<double> elapsed;
                        if ( has_possibleInterrupt && has_possibleInterrupt_dynamic ){
                            std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                            std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                            if ( e1.count() > e2.count() ){
                                elapsed = e1;
                            }
                            else{
                                elapsed = e2;
                            }
                        }
                        else if ( has_possibleInterrupt )
                            elapsed = obsStaticStartTime - targetFindingStartTime;
                        else if ( has_possibleInterrupt_dynamic )
                            elapsed = obsDynamicStartTime - targetFindingStartTime;
            
                        auto start_time_t = std::chrono::system_clock::to_time_t(
                            std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
                        );
                        //  const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - lookAroundStartTime;
                        std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
                        std::cout <<" Interrupted time(due to dodging) after: " << elapsed.count() << "seconds(s)" << std::endl;
                        LookAroundElapsed += elapsed.count();
                        lookAroundStartTime = std::chrono::high_resolution_clock::now();
                        if ( has_possibleInterrupt )
                            has_possibleInterrupt = false;
                        else if ( has_possibleInterrupt_dynamic )
                            has_possibleInterrupt_dynamic = false;              

                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(validRangeMutex);
                            cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                        }
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.LookAroundElapsed = LookAroundElapsed;
                            cur_lookAroundStruct.lookAroundStartTime = lookAroundStartTime;
                        }   
                        {
                            std::lock_guard<std::mutex> lck(dynamicObsMutex);
                            cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                            cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                        }   
                    }
                    
                    // std::cout <<" Record target with angle dev: " << std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state_yaw ) ) << ", and vector size: " << angleFrontState_vec.size() << std::endl;
                    // Record the angle and front
                    angleFrontState state;
                    state.front = front;
                    state.angle = wrap_angle(cur_state_yaw);
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                    
                    state.front = rear;
                    state.angle = wrap_angle(cur_state_yaw + M_PI);
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                    state.front = left;
                    state.angle = wrap_angle(cur_state_yaw + M_PI / 2.0f);
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                    state.front = right;
                    state.angle = wrap_angle(cur_state_yaw - M_PI / 2.0f);
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                    // std::cout <<" Add to vector... " << std::endl;
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                    }  
                }
                else{
                    // Stop the turning action
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);

                    if ( cur_lookAroundState.nTimer < 2 ){
                        cur_lookAroundState.clearPathCheckStarted = false;
                        cur_lookAroundState.nTimer += 1;
                        
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.cur_lookAroundState.clearPathCheckStarted = cur_lookAroundState.clearPathCheckStarted;
                            cur_lookAroundStruct.cur_lookAroundState.nTimer = cur_lookAroundState.nTimer;
                        }   
                        continue;
                    }
                    else{
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        cur_lookAroundState.nTimer = 0;
                        
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.cur_lookAroundState.nTimer = cur_lookAroundState.nTimer;
                        }   
                    }

                    // Sort the angle array first 
                    if ( cur_lookAroundState.smallToBig == false ){
                        std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                            if( a.front != b.front )
                                return a.front > b.front;
                            return a.angle < b.angle;
                        });
                        cur_lookAroundState.smallToBig = true;
                    }
                    else{
                        std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                            if( a.front != b.front )
                                return a.front < b.front;
                            return a.angle < b.angle;
                        });
                        cur_lookAroundState.smallToBig = false;
                    }

                    // Check for clear path
                    std::cout <<" Start path checking with angle vector size: " << angleFrontState_vec.size() << std::endl;
                    bool hasObOnPath = false; 
                    for ( const auto& pair_cand : angleFrontState_vec ){
                        if ( pair_cand.front <= safe_endreach_dist + pair_cand.front / 5 ){
                            continue;
                        }

                        if ( std::abs( angleDifference( cur_lookAroundState.preAngle, pair_cand.angle ) ) <= 20.0f / 180 * M_PI ){
                            continue;
                        }

                        float angMin = std::abs( std::atan2( 0.1f, pair_cand.front ) );
                        hasObOnPath = false;
                        for ( const auto& pair_to_compare : angleFrontState_vec ){
                            float angDev = std::abs( angleDifference( pair_to_compare.angle, pair_cand.angle ) );
                            if ( angDev <= angMin ){
                                if ( pair_to_compare.front * std::cos( angDev ) < pair_cand.front - safe_endreach_dist ){
                                    hasObOnPath = true;
                                    break;
                                }
                            }
                            else if ( angDev <= 45.0f / 180.0f * M_PI && angDev > angMin ){
                                if ( pair_to_compare.front * std::sin( angDev ) <= 0.1f ){
                                    hasObOnPath = true;
                                    break;
                                }
                            }                        
                        }

                        if ( hasObOnPath == false ){
                            // Found the path, turn to that angle
                            cur_lookAroundState.targetAngle = pair_cand.angle;

                            // Try to turn to the angle
                            // std::cout <<" Current angle: " << cur_state_yaw << std::endl;
                            // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                            // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                            // std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) << std::endl;
                            float angTurn = angleDifference( cur_state_yaw, pair_cand.angle ) + 5.0f / 180.0f * M_PI;
                            if ( angleDifference( cur_state_yaw, pair_cand.angle ) < 0.0f ){
                                angTurn -= 10.0f / 180.0f * M_PI;
                            }
                            Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1);
                            std::cout <<" Found a path to go to and start turning to target angle with target: " << pair_cand.angle << std::endl;
                            cur_lookAroundState.turnStarted = true;
                        
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(lookAroundMutex);
                                cur_lookAroundStruct.cur_lookAroundState.targetAngle = cur_lookAroundState.targetAngle;
                                cur_lookAroundStruct.cur_lookAroundState.turnStarted = cur_lookAroundState.turnStarted;
                            }                               
                            break;
                        }
                    }

                    // If turning still not starting, we return to previous angle
                    if ( cur_lookAroundState.turnStarted == false ){
                        // Found the path, turn to that angle
                        cur_lookAroundState.targetAngle = cur_lookAroundState.preAngle;

                        // Try to turn to the angle
                        // std::cout <<" Current angle: " << cur_state_yaw << std::endl;
                        // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                        // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                        // std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) << std::endl;
                        float angTurn = angleDifference( cur_state_yaw, cur_lookAroundState.preAngle ) + 5.0f / 180.0f * M_PI;
                        if ( angleDifference( cur_state_yaw, cur_lookAroundState.preAngle ) < 0.0f ){
                            angTurn -= 10.0f / 180.0f * M_PI;
                        }
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                        std::cout <<" No path to go to so start turning to the previous target angle with target: " << cur_lookAroundState.preAngle << std::endl;
                        cur_lookAroundState.turnStarted = true;
                        
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.cur_lookAroundState.targetAngle = cur_lookAroundState.targetAngle;
                            cur_lookAroundStruct.cur_lookAroundState.turnStarted = cur_lookAroundState.turnStarted;
                        }   
                    }
                }
            }
            else{
                if ( std::abs( angleDifference( cur_lookAroundState.targetAngle, cur_state_yaw ) ) >= 5.0f / 180.0f * M_PI ){
                    if ( has_possibleInterrupt || has_possibleInterrupt_dynamic ){
                        std::cout <<" Some targets occur, so try to turn to the target look up angle again..." << std::endl;
                        float angTurn = angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                        if ( angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                            angTurn -= 10.0f / 180.0f * M_PI;
                        }
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                        on_TURNING_MODE = true;

                        std::chrono::duration<double> elapsed;
                        if ( has_possibleInterrupt && has_possibleInterrupt_dynamic ){
                            std::chrono::duration<double> e1 = obsStaticStartTime - targetFindingStartTime;
                            std::chrono::duration<double> e2 = obsDynamicStartTime - targetFindingStartTime;
                            if ( e1.count() > e2.count() ){
                                elapsed = e1;
                            }
                            else{
                                elapsed = e2;
                            }
                        }
                        else if ( has_possibleInterrupt )
                            elapsed = obsStaticStartTime - targetFindingStartTime;
                        else if ( has_possibleInterrupt_dynamic )
                            elapsed = obsDynamicStartTime - targetFindingStartTime;
            
                        auto start_time_t = std::chrono::system_clock::to_time_t(
                            std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
                        );
                        //  const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - lookAroundStartTime;
                        std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
                        std::cout <<" Interrupted time(due to dodging) after: " << elapsed.count() << "seconds(s)" << std::endl;
                        LookAroundElapsed += elapsed.count();
                        lookAroundStartTime = std::chrono::high_resolution_clock::now();
                        if ( has_possibleInterrupt )
                            has_possibleInterrupt = false;
                        else if ( has_possibleInterrupt_dynamic )
                            has_possibleInterrupt_dynamic = false; 
                    }
                    
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                    }
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.LookAroundElapsed = LookAroundElapsed;
                        cur_lookAroundStruct.lookAroundStartTime = lookAroundStartTime;
                    }       
                    {
                        std::lock_guard<std::mutex> lck(dynamicObsMutex);
                        cur_dynamicObsStruct.has_possibleInterrupt = has_possibleInterrupt;
                        cur_dynamicObsStruct.has_possibleInterrupt_dynamic = has_possibleInterrupt_dynamic;
                    }                  
                    // continue turning
                    // std::cout <<" Keep turning to that angle with current angle: " << cur_state_yaw << std::endl;
                    continue;
                }
                else{
                    // Ready to go to path
                    std::cout <<" Turn to the target angle, ready to go to it..." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));

                    // If current front is not goable
                    float safe_dist{0.0f};
                    if ( front >= 1.0f )
                        safe_dist = safe_endreach_dist + front / 4 + 0.35f;
                    else
                        safe_dist = safe_endreach_dist;
                    if ( front <= safe_dist ){
                        std::cout <<" Turn to the target angle, But current angle is not goable..." << std::endl;
                        angleFrontState state;
                        state.front = front;
                        state.angle = wrap_angle(cur_state_yaw);
                        angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.angleFrontState_vec = angleFrontState_vec;
                        }  

                        // Start to find another way to go to
                        // Sort the angle array first 
                        if ( cur_lookAroundState.smallToBig == false ){
                            std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                                if( a.front != b.front )
                                    return a.front > b.front;
                                return a.angle < b.angle;
                            });
                            cur_lookAroundState.smallToBig = true;
                        }
                        else{
                            std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                                if( a.front != b.front )
                                    return a.front < b.front;
                                return a.angle < b.angle;
                            });
                            cur_lookAroundState.smallToBig = false;
                        }
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.cur_lookAroundState.smallToBig = cur_lookAroundState.smallToBig;
                        }  

                        // Check for clear path
                        std::cout <<" Start path checking..." << std::endl;
                        bool hasObOnPath = false; 
                        bool hasFoundPath = false;
                        for ( const auto& pair_cand : angleFrontState_vec ){
                            if ( pair_cand.front <= safe_endreach_dist + pair_cand.front / 5 ){
                                continue;
                            }

                            if ( std::abs( angleDifference( cur_lookAroundState.preAngle, cur_state_yaw ) ) > 20.0f / 180 * M_PI ){
                                if ( std::abs( angleDifference( cur_lookAroundState.preAngle, pair_cand.angle ) ) <= 20.0f / 180 * M_PI ){
                                    continue;
                                }
                            }

                            if ( std::abs( angleDifference( pair_cand.angle, cur_state_yaw ) ) <= 1.0 / 180.0f * M_PI ){
                                continue;
                            }

                            float angMin = std::abs( std::atan2( 0.1f, pair_cand.front ) );
                            hasObOnPath = false;
                            for ( const auto& pair_to_compare : angleFrontState_vec ){
                                float angDev = std::abs( angleDifference( pair_to_compare.angle, pair_cand.angle ) );
                                if ( angDev <= angMin ){
                                    if ( pair_to_compare.front * std::cos( angDev ) < pair_cand.front - safe_endreach_dist ){
                                        hasObOnPath = true;
                                        break;
                                    }
                                }
                                else if ( angDev <= 45.0f / 180.0f * M_PI && angDev > angMin ){
                                    if ( pair_to_compare.front * std::sin( angDev ) <= 0.1f ){
                                        hasObOnPath = true;
                                        break;
                                    }
                                }                        
                            }

                            if ( hasObOnPath == false && std::abs( angleDifference( cur_lookAroundState.preAngle, cur_state_yaw ) ) > 20.0f / 180 * M_PI ){
                                hasFoundPath = true;
                                // Found the path, turn to that angle
                                cur_lookAroundState.targetAngle = pair_cand.angle;

                                // Try to turn to the angle
                                // std::cout <<" Current angle: " << cur_state_yaw << std::endl;
                                // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                                // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                                // std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) << std::endl;
                                float angTurn = angleDifference( cur_state_yaw, pair_cand.angle) + 5.0f / 180.0f * M_PI;
                                if ( angleDifference( cur_state_yaw, pair_cand.angle ) < 0.0f ){
                                    angTurn -= 10.0f / 180.0f * M_PI;
                                }
                                Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                                std::cout <<" Found a path to go to and start turning to target angle with target: " << pair_cand.angle << std::endl;
                                break;
                            }
                        }

                        // Found the path, turn to that angle
                        if ( hasFoundPath ){                            
                            // Set variables back
                            {
                                std::lock_guard<std::mutex> lck(lookAroundMutex);
                                cur_lookAroundStruct.cur_lookAroundState.targetAngle = cur_lookAroundState.targetAngle;
                            }   
                            continue;
                        }

                        // If turning still not starting, we return to previous angle
                        cur_lookAroundState.targetAngle = cur_lookAroundState.preAngle;

                        // Try to turn to the angle
                        // std::cout <<" Current angle: " << cur_state_yaw << std::endl;
                        // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                        // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                        // std::cout <<" Angle difference: " << angleDifference( cur_state_yaw, cur_lookAroundState.targetAngle ) << std::endl;
                        float angTurn = angleDifference( cur_state_yaw, cur_lookAroundState.preAngle ) + 5.0f / 180.0f * M_PI;
                        if ( angleDifference( cur_state_yaw, cur_lookAroundState.preAngle ) < 0.0f ){
                            angTurn -= 10.0f / 180.0f * M_PI;
                        }
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                        std::cout <<" No path to go to so start turning to the previous target angle with target: " << cur_lookAroundState.preAngle << std::endl;
                        
                        // Set variables back
                        {
                            std::lock_guard<std::mutex> lck(lookAroundMutex);
                            cur_lookAroundStruct.cur_lookAroundState.targetAngle = cur_lookAroundState.targetAngle;
                        }   
                        continue;                        
                    }

                    cur_pathReachingState.pathReadyToGo = true;
                    ori_front = front;

                    // Reset other flags
                    cur_lookAroundState.turnStarted = false;
                    cur_lookAroundState.clearPathCheckStarted = false;
                    cur_lookAroundState.startAngle = -10.0f;
                    on_TURNING_MODE = false;

                    lookAroundEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = lookAroundEndTime - lookAroundStartTime;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundEndTime)
                    );

                    std::cout <<" Look around complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;

                    LookAroundElapsed += elapsed.count();
                    std::cout <<" , average look around elapsed: " << LookAroundElapsed / nlookAroundCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has done look around for " << nlookAroundCount << " times" << std::endl;
            
                    // Set variables back
                    {
                        std::lock_guard<std::mutex> lck(frontReachingMutex);
                        cur_frontReachingStruct.cur_pathReachingState.pathReadyToGo = cur_pathReachingState.pathReadyToGo;
                    }  
                    {
                        std::lock_guard<std::mutex> lck(lookAroundMutex);
                        cur_lookAroundStruct.ori_front = cur_lookAroundStruct.ori_front;
                        cur_lookAroundStruct.cur_lookAroundState.turnStarted = cur_lookAroundState.turnStarted;
                        cur_lookAroundStruct.cur_lookAroundState.clearPathCheckStarted = cur_lookAroundState.clearPathCheckStarted;
                        cur_lookAroundStruct.cur_lookAroundState.startAngle = cur_lookAroundState.startAngle;
                        cur_lookAroundStruct.lookAroundEndTime = lookAroundEndTime;
                        cur_lookAroundStruct.LookAroundElapsed = LookAroundElapsed;
                    }   
                    {
                        std::lock_guard<std::mutex> lck(validRangeMutex);
                        cur_validRangeStruct.on_TURNING_MODE = on_TURNING_MODE;
                    }
                }
            }   
        }        
    });

    // Check Landing in the main loop
    while( od4->isRunning() ){ 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Variables here     
        float front{0};
        float rear{0};
        float left{0};
        float right{0};
        float cur_state_yaw{0.0f};
        float cur_state_battery_state{0.0f};
        float dist_target{-1.0f};
        float dist_obs{-1.0f};
        float dist_chpad{-1.0f};
        float aimDirection_target{-4.0f};
        float aimDirection_obs{-4.0f};
        float aimDirection_chpad{-4.0f};
        float closeBallTimer{0.0f};
        int16_t closeBallCount{0}; 
        float closeStaticObsTimer{0.0f};
        int16_t closeStaticObsCount{0}; 
        int16_t nTargetTimer{0};
        int16_t is_chpad_found{0}; 
        {
            front = cur_sensorReadStruct.front;
            rear = cur_sensorReadStruct.rear;
            left = cur_sensorReadStruct.left;
            right = cur_sensorReadStruct.right;
            cur_state_yaw = cur_sensorReadStruct.cur_state_yaw;
            cur_state_battery_state = cur_sensorReadStruct.cur_state_battery_state;
            dist_target = cur_sensorReadStruct.dist_target;
            dist_obs = cur_sensorReadStruct.dist_obs;
            dist_chpad = cur_sensorReadStruct.dist_chpad;
            aimDirection_target = cur_sensorReadStruct.aimDirection_target;
            aimDirection_obs = cur_sensorReadStruct.aimDirection_obs;
            aimDirection_chpad = cur_sensorReadStruct.aimDirection_chpad;
            closeBallTimer = cur_sensorReadStruct.closeBallTimer;
            closeBallCount = cur_sensorReadStruct.closeBallCount;
            closeStaticObsTimer = cur_sensorReadStruct.closeStaticObsTimer;
            closeStaticObsCount = cur_sensorReadStruct.closeStaticObsCount; 
            nTargetTimer = cur_sensorReadStruct.nTargetTimer;
            is_chpad_found = cur_sensorReadStruct.is_chpad_found;
        }

        // Variables for constant
        float safe_endreach_ultimate_dist{0.0f};
        float safe_endreach_dist{0.0f};
        float safe_endreach_LR_dist{0.0f};
        int16_t nTargetCount{0}; // 2 for maze and 3 for rooms 
        {
            safe_endreach_ultimate_dist = cur_constVarStruct.safe_endreach_ultimate_dist;
            safe_endreach_dist = cur_constVarStruct.safe_endreach_dist;
            safe_endreach_LR_dist = cur_constVarStruct.safe_endreach_LR_dist;
            nTargetCount = cur_constVarStruct.nTargetCount;
        }

        // Variables for stuck escape check
        int nFrontReachingTimer = 0;
        int nStuckEscapeCount = 0;
        float pre_front = -1.0f;
        float front_dev = -1.0f;
        {
            std::lock_guard<std::mutex> lck(stuckEscapeMutex);
            nFrontReachingTimer = cur_stuckEscapeStruct.nFrontReachingTimer;
            nStuckEscapeCount = cur_stuckEscapeStruct.nStuckEscapeCount;
            pre_front = cur_stuckEscapeStruct.pre_front;
            front_dev = cur_stuckEscapeStruct.front_dev;
        } 

        // Variables for valid range
        std::vector<distPathState> distPathstate_vec;
        ValidWay cur_validWay = {-1.0f, -1.0f, -1.0f};
        bool on_GoTO_MODE = false;
        bool on_TURNING_MODE = false;
        float preFront_togo{-1.0f};
        {
            std::lock_guard<std::mutex> lck(validRangeMutex);
            distPathstate_vec = cur_validRangeStruct.distPathstate_vec;
            cur_validWay = cur_validRangeStruct.cur_validWay;
            on_GoTO_MODE = cur_validRangeStruct.on_GoTO_MODE;
            on_TURNING_MODE = cur_validRangeStruct.on_TURNING_MODE;
            preFront_togo = cur_validRangeStruct.preFront_togo;
        }

        // Variables for static obstacles avoidance
        float cur_distToMove{0.0f};
        int time_toMove = 1;
        preDist cur_preDist = {-1.0f, -1.0f};
        ReachEndState cur_reachEndState = { false, false, false, false };
        bool staticDodgeLeft = false;
        bool staticDodgeRight = false;
        bool isCloseToStaticObs = false;
        int nObsStaticCount = 0;
        double ObsStaticElapsed = 0.0f;
        auto obsStaticStartTime = std::chrono::high_resolution_clock::now();
        auto obsStaticEndTime = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lck(staticObsMutex);
            cur_distToMove = cur_staticObsStruct.cur_distToMove;
            time_toMove = cur_staticObsStruct.time_toMove;
            cur_preDist = cur_staticObsStruct.cur_preDist;
            cur_reachEndState = cur_staticObsStruct.cur_reachEndState;
            staticDodgeLeft = cur_staticObsStruct.staticDodgeLeft;
            staticDodgeRight = cur_staticObsStruct.staticDodgeRight;
            isCloseToStaticObs = cur_staticObsStruct.isCloseToStaticObs;
            nObsStaticCount = cur_staticObsStruct.nObsStaticCount;
            ObsStaticElapsed = cur_staticObsStruct.ObsStaticElapsed;
            obsStaticStartTime = cur_staticObsStruct.obsStaticStartTime;
            obsStaticEndTime = cur_staticObsStruct.obsStaticEndTime;
        } 

        float dodgeDist{0.0f};
        float dodgeDist_UP{0.0f};
        bool has_dodgeToRear = false;
        DodgeType cur_dodgeType = DODGE_NONE;
        obsState cur_obsState = { -1.0f, -1.0f };
        bool has_possibleInterrupt = false;
        bool has_possibleInterrupt_dynamic = false;
        bool has_InterruptNeedToReDo = false;
        bool has_InterruptNeedToReDo_dynamic = false;
        int nObsDynamicCount = 0;
        double ObsDynamicElapsed = 0.0f;
        auto obsDynamicStartTime = std::chrono::high_resolution_clock::now();
        auto obsDynamicEndTime = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lck(dynamicObsMutex);
            dodgeDist = cur_dynamicObsStruct.dodgeDist;
            dodgeDist_UP = cur_dynamicObsStruct.dodgeDist_UP;
            has_dodgeToRear = cur_dynamicObsStruct.has_dodgeToRear;
            cur_dodgeType = cur_dynamicObsStruct.cur_dodgeType;
            cur_obsState = cur_dynamicObsStruct.cur_obsState;
            has_possibleInterrupt = cur_dynamicObsStruct.has_possibleInterrupt;
            has_possibleInterrupt_dynamic = cur_dynamicObsStruct.has_possibleInterrupt_dynamic;
            has_InterruptNeedToReDo = cur_dynamicObsStruct.has_InterruptNeedToReDo;
            has_InterruptNeedToReDo_dynamic = cur_dynamicObsStruct.has_InterruptNeedToReDo_dynamic;
            nObsDynamicCount = cur_dynamicObsStruct.nObsDynamicCount;
            ObsDynamicElapsed = cur_dynamicObsStruct.ObsDynamicElapsed;
            obsDynamicStartTime = cur_dynamicObsStruct.obsDynamicStartTime;
            obsDynamicEndTime = cur_dynamicObsStruct.obsDynamicEndTime;
        }   

        // Variables for target finding
        targetCheckState cur_targetCheckState = {false, false, false, -1.0f, 100.0f / 180.0f * M_PI, -1.0f, -1.0f, -1.0f};
        float start_turning_angle{0.0f};
        float dist_to_reach{0.0f};
        float aimDirection_to_reach{0.0f};
        int nTargetFindingCount = 0;
        double TargetFindingElapsed = 0.0f;
        auto targetFindingStartTime = std::chrono::high_resolution_clock::now();
        auto targetFindingEndTime = std::chrono::high_resolution_clock::now(); 
        {
            std::lock_guard<std::mutex> lck(targetFindingMutex);
            cur_targetCheckState = cur_targetFindingStruct.cur_targetCheckState;
            start_turning_angle = cur_targetFindingStruct.start_turning_angle;
            dist_to_reach = cur_targetFindingStruct.dist_to_reach;
            aimDirection_to_reach = cur_targetFindingStruct.aimDirection_to_reach;
            nTargetFindingCount = cur_targetFindingStruct.nTargetFindingCount;
            TargetFindingElapsed = cur_targetFindingStruct.TargetFindingElapsed;
            targetFindingStartTime = cur_targetFindingStruct.targetFindingStartTime;
            targetFindingEndTime = cur_targetFindingStruct.targetFindingEndTime;
        }

        // Variables for front reaching
        pathReachingState cur_pathReachingState = {false, false, -1.0f};
        int nfrontReachingCount = 0;
        double FrontReachingElapsed = 0.0f;
        auto frontReachingStartTime = std::chrono::high_resolution_clock::now();
        auto frontReachingEndTime = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lck(frontReachingMutex);
            cur_pathReachingState = cur_frontReachingStruct.cur_pathReachingState;
            nfrontReachingCount = cur_frontReachingStruct.nfrontReachingCount;
            FrontReachingElapsed = cur_frontReachingStruct.FrontReachingElapsed;
            frontReachingStartTime = cur_frontReachingStruct.frontReachingStartTime;
            frontReachingEndTime = cur_frontReachingStruct.frontReachingEndTime;
        }

        // Variables for looking around
        std::vector<angleFrontState> angleFrontState_vec;
        lookAroundState cur_lookAroundState = {false, false, false, -1.0f, -10.0f, -1.0f, 0};
        float ori_front{0.0f};
        int nlookAroundCount = 0;
        double LookAroundElapsed = 0.0f;
        auto lookAroundStartTime = std::chrono::high_resolution_clock::now();
        auto lookAroundEndTime = std::chrono::high_resolution_clock::now(); 
        {
            std::lock_guard<std::mutex> lck(lookAroundMutex);
            angleFrontState_vec = cur_lookAroundStruct.angleFrontState_vec;
            cur_lookAroundState = cur_lookAroundStruct.cur_lookAroundState;
            ori_front = cur_lookAroundStruct.ori_front;
            nlookAroundCount = cur_lookAroundStruct.nlookAroundCount;
            LookAroundElapsed = cur_lookAroundStruct.LookAroundElapsed;
            lookAroundStartTime = cur_lookAroundStruct.lookAroundStartTime;
            lookAroundEndTime = cur_lookAroundStruct.lookAroundEndTime;
        }

        /*
            Landing check and target decision
        */
        dist_to_reach = dist_target;
        aimDirection_to_reach = aimDirection_target;
        if ( cur_state_battery_state <= homing_batterythreshold || nTargetTimer >= nTargetCount ){
            //  std::cout <<" Change to find the charging pad: " << nTargeCount << std::endl;
            dist_to_reach = dist_chpad;
            aimDirection_to_reach = aimDirection_chpad;

            // Find the charging pad, stop and do landing
            if ( (is_chpad_found == 1||(dist_to_reach * std::cos( aimDirection_to_reach ) <= 30.0f && dist_to_reach <= 150.0f && dist_to_reach != -1.0f)) ){
                if ( cur_state_battery_state <= homing_batterythreshold || nTargetTimer >= nTargetCount ) {
                    Landing(od4, 0.0f, 3);
                    Stopping(od4);
                    if ( nTargetTimer >= nTargetCount && is_chpad_found == 1 )
                        std::cout <<" Successfully do landing and stopping with all targets found..." << std::endl;
                    else if ( cur_state_battery_state <= homing_batterythreshold )
                        std::cout <<" Do landing and stopping with low battery..." << std::endl;

                    // Record the end time
                    isTerminateThread = true;
                    taskEndTime = std::chrono::high_resolution_clock::now();
                    const std::chrono::duration<double> elapsed = taskEndTime - taskStartTime;

                    auto start_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(taskStartTime)
                    );
                    auto end_time_t = std::chrono::system_clock::to_time_t(
                        std::chrono::time_point_cast<std::chrono::system_clock::duration>(taskEndTime)
                    );

                    std::cout <<" Task complete with start time: " << std::ctime(&start_time_t) << std::endl;
                    std::cout <<" , end time: " << std::ctime(&end_time_t) << std::endl;
                    std::cout <<" , task elapsed: " << elapsed.count() << " seconds(s)" << std::endl;
                    std::cout <<" , average obs static dodging elapsed: " << ObsStaticElapsed / nObsStaticCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has dodged obs static for " << nObsStaticCount << " times" << std::endl;
                    std::cout <<" , average obs dynamic dodging elapsed: " << ObsDynamicElapsed / nObsDynamicCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has dodged obs dynamic for " << nObsDynamicCount << " times" << std::endl;
                    std::cout <<" , average target finding elapsed: " << TargetFindingElapsed / nTargetFindingCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has done target finding for " << nTargetFindingCount << " times" << std::endl;
                    std::cout <<" , average front reaching elapsed: " << FrontReachingElapsed / nfrontReachingCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has done front reaching for " << nfrontReachingCount << " times" << std::endl;
                    std::cout <<" , average look around elapsed: " << LookAroundElapsed / nlookAroundCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has done look around for " << nlookAroundCount << " times" << std::endl;
                    std::cout <<" , average close to ball elapsed: " << closeBallTimer / closeBallCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has closed to ball for " << closeBallCount << " times" << std::endl;
                    std::cout <<" , average close to static obs elapsed: " << closeStaticObsTimer / closeStaticObsCount << " seconds(s)" << std::endl;
                    std::cout <<" , so far has closed to static obs for " << closeStaticObsCount << " times" << std::endl;
                    std::cout <<" , so far has escaped from stucks for " << nStuckEscapeCount << " times" << std::endl;
                    break;
                }
            }
        }
        {
            std::lock_guard<std::mutex> lck(targetFindingMutex);
            cur_targetFindingStruct.dist_to_reach = dist_to_reach;
            cur_targetFindingStruct.aimDirection_to_reach = aimDirection_to_reach;
        }
    }    

    // Detach every thread
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    ValidRangeCheckTask.detach();
    StuckEscapeTask.detach();
    StaticObsDodgeTask.detach();
    DynamicObsDodgeTask.detach();
    TargetFindingTask.detach();
    FrontReachingTask.detach();
    LookAroundTask.detach();

    return 0;
}