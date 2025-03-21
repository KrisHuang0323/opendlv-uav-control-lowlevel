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


void Takeoff(cluon::OD4Session &od4, float height, int duration){
    std::cout << "Taking off to height: " << height << std::endl;        
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Goto(cluon::OD4Session &od4, float x, float y, float z, float yaw, int duration, int relative = 1, bool isdelay = false){
    std::cout << "Go to position : x: "<< x << " ,y: " << y << " ,z: " << z << " , yaw: " << yaw << std::endl;
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
    std::cout << "Landing to height: " << height << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
}

void Stopping(cluon::OD4Session &od4){
    std::cout << "Stopping." << std::endl;
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

/*
Input: 
- front/rear/left/right IR sensor distance
- purple ball position(If some exist, to dodge it)
- green ball position(For goal)
- charging base position

Output:
- Corresponding actions
*/
int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << "You should include the cid to start communicate in OD4Session" << std::endl;
        return retCode;
    }

    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

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
    float takeoff_batterythreshold = 3.8f;

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

    // Variables for static obstacles avoidance
    float safe_endreach_dist = 0.3;
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
    struct Range {
        float left;
        float right;
    };
    Range ori_dodgeRange = {-1.0f, -1.0f};
    bool has_recordOriRange = false;
    enum DodgeType {
        DODGE_LEFT,
        DODGE_RIGHT,
        DODGE_REAR,
        DODGE_UP,
        DODGE_NONE
    };
    DodgeType cur_dodgeType = DODGE_NONE;
    bool has_possibleInterrupt = false;

    // Variables for front reaching
    struct pathReachingState {
        bool pathReadyToGo;
        bool pathOnGoing;
        float startFront;
    };
    pathReachingState cur_pathReachingState = {false, false, -1.0f};

    // Variables for target finding
    struct targetCheckState {
        bool clearPathCheckStarted;
        bool turnStarted;
        float startAngle;
        float targetAngle;
    };
    targetCheckState cur_targetCheckState = {false, false, -1.0f, -1.0f};
    float start_turning_angle{0.0f};

    // Variables for homing
    float homing_batterythreshold = 3.4f;

    // Variables for looking around
    struct angleFrontState {
        float angle;
        float front;
    };
    std::vector<angleFrontState> angleFrontState_vec;
    struct lookAroundState {
        bool clearPathCheckStarted;
        bool turnStarted;
        float preAngle;
        float startAngle;
        float targetAngle;
    };
    lookAroundState cur_lookAroundState = {false, false, -1.0f, -1.0f, -1.0f};
    float ori_front{0.0f};
    float ori_rear{0.0f};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution (0, 1);

    while (od4.isRunning()) {
        // Sleep for 10 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /*
            Takeoff
            - Do it before start any goto action
            - Check if the battery state is acceptable to take off
        */
        if ( hasTakeoff == false ){
            if ( cur_state.battery_state > takeoff_batterythreshold ){
                Takeoff(od4, 1.0f, 3);
                hasTakeoff = true;
            }
            else{
                std::cout <<" Battery is too low for taking off..." << std::endl;
                continue;
            }
        }

        /*
            Refresh valid left/right/rear on the way
        */
        if ( distPathstate_vec.size() > 0 ){
            float cur_dist_onPath = front - ori_front;
            float cur_dist_onPath_rear = rear - ori_front;
            cur_validWay.toRear += cur_dist_onPath_rear;
            for ( auto& state : distPathstate_vec ){
                if ( std::abs( state.dist_valid_onPath - cur_dist_onPath ) <= 0.01f ){
                    continue;
                }

                if ( state.dist_valid_devPath <= 0.0f ){
                    state.dist_valid_devPath = -right;
                }
                else{
                    state.dist_valid_devPath = left; 
                }
            }
        }

        /*
            Valid direction check
            - Check from the range recording array
        */
        // Reset valid way array 
        cur_validWay.toLeft = -1.0f;
        cur_validWay.toRight = -1.0f;

        // Start valid way checking
        if ( distPathstate_vec.size() > 0 ){
            float cur_dist_onPath = front - ori_front;
            float toLeft{-1.0f};
            float toRight{-1.0f};
            for ( auto& state : distPathstate_vec ){
                // While some path in Crazyflie range
                if ( std::abs( state.dist_valid_onPath - cur_dist_onPath ) > 0.1f ){
                    continue;
                }

                if ( state.dist_valid_devPath > 0.0f ){
                    if ( std::abs( state.dist_valid_devPath ) > toLeft ){
                        toLeft = std::abs( state.dist_valid_devPath );
                    }
                }
                else{
                    if ( std::abs( state.dist_valid_devPath ) > toRight ){
                        toRight = std::abs( state.dist_valid_devPath );
                    }
                }
            }

            // Set current valid way to the state
            cur_validWay.toLeft = toLeft;
            cur_validWay.toRight = toRight;
            // std::cout <<" Valid way checking start... with left " << cur_validWay.toLeft << ", with right: " << cur_validWay.toRight << std::endl;
        }

        /*
            Obstacle Avoidance for walls or static obstacles
            - Get the range from rangefinder
            - Check whether some ranges reach ends
        */
       if ( front <= safe_endreach_dist + cur_distToMove / time_toMove + 0.1f ){
            if ( cur_pathReachingState.pathOnGoing ){
                std::cout <<" Front end meets limit." << std::endl;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                cur_reachEndState.reachFront = true; 
            }           
        }
        else{
            cur_reachEndState.reachFront = false;
        }

        if ( left <= safe_endreach_LR_dist ){
            if ( cur_pathReachingState.pathOnGoing == false && cur_dodgeType == DODGE_NONE ){
                cur_preDist.left = -1.0f;
            }
            else{
                if ( cur_preDist.left == -1.0f || cur_preDist.left <= left ){
                    cur_preDist.left = left;
                    std::cout <<" Record left: "<< cur_preDist.left << std::endl;
                }
                else{
                    std::cout <<" Cur left: "<< left << std::endl;
                    std::cout <<" Cur toRight: "<< cur_validWay.toRight << std::endl;
                    if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f && dist_obs == -1.0f ){
                        std::cout <<" Left end meets limit with right direction dodge..." << std::endl;
                        Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                        has_possibleInterrupt = true;
                        continue;
                    }
                    else{
                        std::cout <<" Left end meets limit." << std::endl;
                        Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                        cur_reachEndState.reachLeft = true;
                    }
                }
            }
        }
        else{ 
            cur_reachEndState.reachLeft = false;
        }

        if ( right <= safe_endreach_LR_dist ){
            if ( cur_pathReachingState.pathOnGoing == false && cur_dodgeType == DODGE_NONE ){
                cur_preDist.right = -1.0f;
            }
            else{
                if ( cur_preDist.right == -1.0f || cur_preDist.right <= right ){
                    cur_preDist.right = right;
                    std::cout <<" Record right: "<< cur_preDist.right << std::endl;
                }
                else{
                    std::cout <<" Cur right: "<< right << std::endl;
                    std::cout <<" Cur toLeft: "<< cur_validWay.toLeft << std::endl;
                    if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f && dist_obs == -1.0f ){
                        std::cout <<" Right end meets limit with left direction dodge..." << std::endl;
                        Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                        has_possibleInterrupt = true;
                        continue;
                    }
                    else{
                        std::cout <<" Right end meets limit." << std::endl;
                        Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                        cur_reachEndState.reachRight = true;
                    }
                }
            }
        }
        else{
            cur_reachEndState.reachRight = false;
        }

        if ( rear <= safe_endreach_dist ){
            if ( cur_dodgeType != DODGE_NONE ){
                std::cout <<" Rear end meets limit." << std::endl;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                cur_reachEndState.reachRear = true;
            }          
        }
        else{
            cur_reachEndState.reachRear = false;
        }

        /*
            Obstacle Avoidance for dynamic obstacles
            - Check the obstacle in the vision
            - Use current valid way to dodge away from the obstacle
        */
        if ( dist_obs <= 250.0f && dist_obs > -1.0f ){    // Means that some obstacles approach
            // Record original range
            if ( has_recordOriRange == false ){
                ori_dodgeRange.left = left;
                ori_dodgeRange.right = right;
                has_recordOriRange = true;
            }

            // While some ends are reached, simply fly up until the obstacle go away
            bool hasReachEnd = false;
            if ( cur_reachEndState.reachFront || cur_reachEndState.reachLeft || cur_reachEndState.reachRight || cur_reachEndState.reachRear ){
                hasReachEnd = true;
            }
            if ( hasReachEnd && cur_dodgeType == DODGE_NONE ){
                std::cout <<" Some ends meet limit go up... " << std::endl; 
                Goto(od4, 0.0f, 0.0f, 0.3f, 0.0f, 0, 1, true);
                cur_dodgeType = DODGE_UP;
                continue;
            }

            // Check if the obstacle is on the left or right side
            if ( aimDirection_obs >= 0.0f ){
                // If we have dodge away, simply wait and do nothing
                if ( cur_dodgeType != DODGE_REAR && cur_dodgeType != DODGE_NONE){
                    continue;
                }

                // If right side has valid way, dodge to it
                if ( cur_validWay.toRight >= safe_endreach_dist + 0.2f ){
                    std::cout <<" Try to dodge to the right..." << std::endl;
                    Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                    cur_dodgeType = DODGE_RIGHT;
                    continue;
                }
                else if ( cur_validWay.toLeft >= safe_endreach_dist + 0.4f ){
                    std::cout <<" Try to dodge to the left..." << std::endl;
                    Goto(od4, - 0.4f * std::sin( cur_state.yaw ), 0.4f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                    cur_dodgeType = DODGE_LEFT;
                    continue;
                }
                else if ( cur_validWay.toRear >= safe_endreach_dist + 0.2f ){
                    std::cout <<" Try to dodge to the rear..." << std::endl;
                    Goto(od4, - 0.2f * std::cos( cur_state.yaw ), - 0.2f * std::sin( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                    cur_dodgeType = DODGE_REAR;
                    has_recordOriRange = false;
                    continue;
                }
                else{
                    std::cout <<" No valid way to dodge to, try to fly up..." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.3f, 0.0f, 0, 1, true);
                    cur_dodgeType = DODGE_UP;
                    continue;
                }

            }
            else{
                // If we have dodge away, simply wait and do nothing
                if ( cur_dodgeType != DODGE_REAR && cur_dodgeType != DODGE_NONE){
                    continue;
                }

                // If left side has valid way, dodge to it
                if ( cur_validWay.toLeft >= safe_endreach_dist + 0.2f ){
                    std::cout <<" Try to dodge to the left..." << std::endl;
                    Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                    cur_dodgeType = DODGE_LEFT;
                    continue;
                }
                else if ( cur_validWay.toRight >= safe_endreach_dist + 0.4f ){
                    std::cout <<" Try to dodge to the right..." << std::endl;
                    Goto(od4, 0.4f * std::sin( cur_state.yaw ), - 0.4f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                    cur_dodgeType = DODGE_RIGHT;
                    continue;
                }
                else if ( cur_validWay.toRear >= safe_endreach_dist + 0.2f ){
                    std::cout <<" Try to dodge to the rear..." << std::endl;
                    Goto(od4, - 0.2f * std::cos( cur_state.yaw ), - 0.2f * std::sin( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying rear to dodge
                    cur_dodgeType = DODGE_REAR;
                    has_recordOriRange = false;
                    continue;
                }
                else{
                    std::cout <<" No valid way to dodge to, try to fly up..." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.3f, 0.0f, 0, 1, true);
                    cur_dodgeType = DODGE_UP;
                    continue;
                }
            }
        }
        else if ( cur_dodgeType != DODGE_NONE ){
            // Go back to the original position while the obstacle is gone
            float devPath = left - ori_dodgeRange.left;
            if ( cur_dodgeType == DODGE_UP ){
                std::cout <<" No obs, try to fly down..." << std::endl;
                Goto(od4, 0.0f, 0.0f, -0.3f, 0.0f, 0, 1, true);
            }
            else{
                if ( devPath <= 0.0f ){
                    std::cout <<" No obs, try to fly back to right..." << std::endl;
                    Goto(od4, std::abs(devPath) * std::sin( cur_state.yaw ), - std::abs(devPath) * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true); 
                }
                else{
                    std::cout <<" No obs, try to fly back to left..." << std::endl;
                    Goto(od4, - devPath * std::sin( cur_state.yaw ), devPath * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);  
                }
            }

            // Reset flags
            cur_dodgeType = DODGE_NONE; 
            has_recordOriRange = false;
            has_possibleInterrupt = true;
        }

        /*
            Front reaching
            - Go to the path straightforwardly if permitted
        */
        // Switch between target and charging pad according to battery state 
        float dist_to_reach = dist_target;
        float aimDirection_to_reach = aimDirection_target;
        if ( cur_state.battery_state <= homing_batterythreshold ){
            dist_to_reach = dist_chpad;
            aimDirection_to_reach = aimDirection_chpad;
        }

        // If meets the front/left/right end, reset all flags
        if ( cur_reachEndState.reachFront == true || cur_reachEndState.reachLeft == true || cur_reachEndState.reachRight == true ){
            if ( cur_pathReachingState.pathReadyToGo || cur_pathReachingState.pathOnGoing ){
                std::cout <<" Reach end stop going with front: " << cur_reachEndState.reachFront << ", left: " << cur_reachEndState.reachLeft << ", right: " << cur_reachEndState.reachRight << std::endl;
                cur_pathReachingState.pathReadyToGo = false;
                cur_pathReachingState.pathOnGoing = false;
                cur_targetCheckState.targetAngle = -1.0f;
            }
        }

        // If being interrupted, try to go to the original path again
        if ( has_possibleInterrupt && cur_pathReachingState.pathOnGoing ){
            std::cout <<" Being interrupted and try to go again..." << std::endl;
            cur_pathReachingState.pathOnGoing = false;
            has_possibleInterrupt = false;
        }

        // Go to path
        if ( cur_pathReachingState.pathReadyToGo ){
            if ( cur_pathReachingState.pathOnGoing == false ){
                std::cout <<" Start go to action..." << std::endl;
                cur_distToMove = front;
                // if ( cur_state.battery_state <= homing_batterythreshold ){
                //     cur_distToMove = 0.8f;
                // }
                time_toMove = 5;
                Goto(od4, cur_distToMove * std::cos( cur_state.yaw ), cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);
                cur_pathReachingState.pathOnGoing = true;
                cur_pathReachingState.startFront = front;
            }
            else{
                if ( dist_to_reach > -1.0f && dist_to_reach <= 80.0f ){
                    // Reach the target, stop current action
                    std::cout <<" Reach target with target exist.." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);

                    // Reset flags
                    cur_pathReachingState.pathOnGoing = false;
                    cur_pathReachingState.pathReadyToGo = false;
                    cur_targetCheckState.targetAngle = -1.0f;
                }
                else if ( cur_state.battery_state <= homing_batterythreshold && std::abs( cur_pathReachingState.startFront - front ) >= 0.8f && cur_targetCheckState.targetAngle == -1.0f ){
                    // Reach the target, stop current action
                    std::cout <<" Reach target with front distance reached but with homing mode..." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);

                    // Reset flags
                    cur_pathReachingState.pathOnGoing = false;
                    cur_pathReachingState.pathReadyToGo = false;
                }
                else if ( std::abs( cur_pathReachingState.startFront - front ) >= cur_distToMove ){
                    // Reach the target, stop current action
                    std::cout <<" Reach target with front distance reached..." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);

                    // Reset flags
                    cur_pathReachingState.pathOnGoing = false;
                    cur_pathReachingState.pathReadyToGo = false;
                    cur_targetCheckState.targetAngle = -1.0f;
                }
            }
            continue;
        }
        
        
        /*
            Target finding
            - Check that whether there has green ball or chpad as target
            - If green ball exist, check that the crazyflie has a clear path to it
            - Use the dist/aimDirection from last section
        */
        // If Interrupted
        if ( has_possibleInterrupt ){
            if ( cur_targetCheckState.turnStarted || cur_targetCheckState.clearPathCheckStarted || cur_targetCheckState.targetAngle != -1.0f ){
                std::cout <<" Possible interruption to reset target reaching..." << std::endl;
                cur_targetCheckState.turnStarted = false;
                cur_targetCheckState.clearPathCheckStarted = false;
                cur_targetCheckState.targetAngle = -1.0f;
                has_possibleInterrupt = false;
            }
        }

        // If some targets found
        if ( cur_targetCheckState.turnStarted || cur_targetCheckState.clearPathCheckStarted || cur_targetCheckState.targetAngle != -1.0f ){
            if ( cur_lookAroundState.turnStarted || cur_lookAroundState.clearPathCheckStarted ){
                std::cout <<" Since some targets found, reset look around..." << std::endl;            
                cur_lookAroundState.turnStarted = false;
                cur_lookAroundState.clearPathCheckStarted = false;
            }
        }

        // Try to find a clear path close to the target
        if ( dist_to_reach > -1.0f || cur_targetCheckState.targetAngle != -1.0f ){
            if ( cur_targetCheckState.clearPathCheckStarted == false ){
                std::cout <<" Found the target, ready to turn to it" << std::endl;
                if ( angleFrontState_vec.size() > 0 ){
                    // Initialize the vector here
                    angleFrontState_vec.clear();
                    distPathstate_vec.clear();
                }
                Goto(od4, 0.0f, 0.0f, 0.0f, 100.0f / 180.0f * M_PI, 3);
                cur_targetCheckState.clearPathCheckStarted = true;
                cur_targetCheckState.startAngle = cur_state.yaw;
                cur_targetCheckState.targetAngle = cur_state.yaw + aimDirection_to_reach;
                continue;
            }
            else if ( cur_targetCheckState.turnStarted == false ){
                if ( std::abs( angleDifference( cur_targetCheckState.startAngle, cur_state.yaw ) ) < 90.0f / 180.0f * M_PI ){
                    // Record the angle and front                    
                    angleFrontState state;
                    state.front = front;
                    state.angle = cur_state.yaw;
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                    
                    state.front = rear;
                    state.angle = cur_state.yaw + M_PI;
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                    state.front = left;
                    state.angle = cur_state.yaw + M_PI / 2.0f;
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                    state.front = right;
                    state.angle = cur_state.yaw - M_PI / 2.0f;
                    angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                }
                else{
                    // Stop the turning action
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    
                    // Sort the angle array first                
                    std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [cur_targetCheckState](const angleFrontState& a, const angleFrontState& b) {
                        if(a.angle != b.angle){
                            float angledev_a = std::abs( angleDifference( a.angle,cur_targetCheckState.targetAngle ) );
                            float angledev_b = std::abs( angleDifference( b.angle, cur_targetCheckState.targetAngle ) );
                            return angledev_a < angledev_b;
                        }
                    });
    
                    // Check for clear path
                    bool hasObOnPath = false; 
                    for ( const auto& pair_cand : angleFrontState_vec ){
                        // if ( pair_cand.front <= safe_endreach_dist ){
                        //     continue;
                        // }

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
    
                            // Try to refresh the rear distance
                            float rearDist{0.0f};
                            for ( const auto& pair : angleFrontState_vec ){
                                float angDev = std::abs( angleDifference( pair.angle, pair_cand.angle ) );
                                if ( angDev <= 90.0f / 180.0f * M_PI ){
                                    continue;;
                                }
                                else{
                                    if ( std::abs( pair.front * std::cos( angDev ) ) > rearDist ){
                                        rearDist = std::abs( pair.front * std::cos( angDev ) );
                                    }
                                }                        
                            }
                            cur_validWay.toRear = rearDist;

                            // Record path related information
                            for ( const auto& pair : angleFrontState_vec ){
                                float angDev = angleDifference( pair_cand.angle, pair.angle );
                                distPathState pstate = { pair.front*std::cos( angDev ), pair.front*std::sin( angDev ) }; 
                                distPathstate_vec.insert( distPathstate_vec.begin(), pstate );                    
                            }
    
                            // Try to turn to the angle
                            float angTurn = angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) + 5.0f / 180.0f * M_PI;
                            if ( angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) < 0.0f ){
                                angTurn -= 10.0f / 180.0f * M_PI;
                            }
                            Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                            cur_targetCheckState.turnStarted = true;
                            break;
                        }
                    }
                } 
                continue;
            }
            else{
                if ( std::abs( angleDifference( cur_targetCheckState.targetAngle, cur_state.yaw ) ) >= 5.0f / 180.0f * M_PI ){
                    // continue turning
                    continue;
                }
                else{
                    // Ready to go to path
                    std::cout <<" Turn to the target angle, ready to go to it..." << std::endl;
                    cur_pathReachingState.pathReadyToGo = true;
                    ori_front = front;
                    ori_rear = rear;
    
                    // Reset other flags
                    cur_targetCheckState.turnStarted = false;
                    cur_targetCheckState.clearPathCheckStarted = false;
                }
                continue;
            }    
        }
        

        /*
            Look around
            - Turn around for 360 degree to see whether target exist
            - In the mean time, record some possible front path to go to
        */
        // If Interrupted
        if ( has_possibleInterrupt ){
            if ( cur_lookAroundState.turnStarted || cur_lookAroundState.clearPathCheckStarted ){
                std::cout <<" Possible interruption to reset look around..." << std::endl;
                cur_lookAroundState.turnStarted = false;
                cur_lookAroundState.clearPathCheckStarted = false;
                has_possibleInterrupt = false;
            }
        }

        // Try to find a clear path close to the target
        // std::cout <<" Run to here with clearPathCheckStarted: " << cur_lookAroundState.clearPathCheckStarted << std::endl;
        if ( cur_lookAroundState.clearPathCheckStarted == false ){
            std::cout <<" No target exist, ready to turn around to find" << std::endl;
            if ( angleFrontState_vec.size() > 0 ){
                // Initialize the vector here
                angleFrontState_vec.clear();
                distPathstate_vec.clear();
            }
            cur_lookAroundState.startAngle = cur_state.yaw;
            cur_lookAroundState.preAngle = cur_state.yaw + M_PI;
            Goto(od4, 0.0f, 0.0f, 0.0f, 90.0f / 180.0f * M_PI, 3);
            cur_lookAroundState.clearPathCheckStarted = true;
        }
        else if ( cur_lookAroundState.turnStarted == false ){
            if ( std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state.yaw ) ) < 90.0f / 180.0f * M_PI ){
                // std::cout <<" Record target with angle dev: " << std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state.yaw ) ) << ", and vector size: " << angleFrontState_vec.size() << std::endl;
                // Record the angle and front
                // Record the angle and front
                angleFrontState state;
                state.front = front;
                state.angle = cur_state.yaw;
                angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                
                state.front = rear;
                state.angle = cur_state.yaw + M_PI;
                angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                state.front = left;
                state.angle = cur_state.yaw + M_PI / 2.0f;
                angleFrontState_vec.insert(angleFrontState_vec.begin(),state);

                state.front = right;
                state.angle = cur_state.yaw - M_PI / 2.0f;
                angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                // std::cout <<" Add to vector... " << std::endl;
            }
            else{
                // Stop the turning action
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);

                // Sort the angle array first 
                if ( distribution(gen) ){
                    std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                        if( a.front != b.front )
                            return a.front > b.front;
                    });
                }
                else{
                    std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                        if( a.front != b.front )
                            return a.front < b.front;
                    });
                }

                // Check for clear path
                std::cout <<" Start path checking..." << std::endl;
                bool hasObOnPath = false; 
                for ( const auto& pair_cand : angleFrontState_vec ){
                    if ( pair_cand.front <= safe_endreach_dist ){
                        continue;
                    }

                    if ( std::abs( angleDifference( cur_lookAroundState.preAngle, pair_cand.angle ) ) <= 10.0f / 180 * M_PI ){
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

                        // Try to refresh the rear distance
                        float rearDist{0.0f};
                        for ( const auto& pair : angleFrontState_vec ){
                            float angDev = std::abs( angleDifference( pair.angle, cur_lookAroundState.targetAngle ) );
                            if ( angDev <= 90.0f / 180.0f * M_PI ){
                                continue;;
                            }
                            else{
                                if ( std::abs( pair.front * std::cos( angDev ) ) > rearDist ){
                                    rearDist = std::abs( pair.front * std::cos( angDev ) );
                                }
                            }                        
                        }
                        cur_validWay.toRear = rearDist;

                        // Record path related information
                        for ( const auto& pair : angleFrontState_vec ){
                            float angDev = angleDifference( cur_lookAroundState.targetAngle, pair.angle );
                            distPathState pstate = { pair.front*std::cos( angDev ), pair.front*std::sin( angDev ) }; 
                            distPathstate_vec.insert( distPathstate_vec.begin(), pstate ); 
                            // std::cout <<" Try to print distance path state with on path: " << pstate.dist_valid_onPath << ", and dev path: " << pstate.dist_valid_devPath << std::endl;                  
                        }

                        // Try to turn to the angle
                        float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                        if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                            angTurn -= 10.0f / 180.0f * M_PI;
                        }
                        Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                        std::cout <<" Found a path to go to and start turning to target angle with target: " << cur_lookAroundState.targetAngle << std::endl;
                        cur_lookAroundState.turnStarted = true;
                        break;
                    }
                }

                // If turning still not starting, we return to previous angle
                if ( cur_lookAroundState.turnStarted == false ){
                    // Found the path, turn to that angle
                    cur_lookAroundState.targetAngle = cur_lookAroundState.preAngle;

                    // Try to refresh the rear distance
                    float rearDist{0.0f};
                    for ( const auto& pair : angleFrontState_vec ){
                        float angDev = std::abs( angleDifference( pair.angle, cur_lookAroundState.targetAngle ) );
                        if ( angDev <= 90.0f / 180.0f * M_PI ){
                            continue;;
                        }
                        else{
                            if ( std::abs( pair.front * std::cos( angDev ) ) > rearDist ){
                                rearDist = std::abs( pair.front * std::cos( angDev ) );
                            }
                        }                        
                    }
                    cur_validWay.toRear = rearDist;

                    // Record path related information
                    for ( const auto& pair : angleFrontState_vec ){
                        float angDev = angleDifference( cur_lookAroundState.targetAngle, pair.angle );
                        distPathState pstate = { pair.front*std::cos( angDev ), pair.front*std::sin( angDev ) }; 
                        distPathstate_vec.insert( distPathstate_vec.begin(), pstate );                    
                    }

                    // Try to turn to the angle
                    float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                    if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                        angTurn -= 10.0f / 180.0f * M_PI;
                    }
                    Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                    std::cout <<" No path to go to so start turning to the previous target angle with target: " << cur_lookAroundState.targetAngle << std::endl;
                    cur_lookAroundState.turnStarted = true;
                }
            }
        }
        else{
            if ( std::abs( angleDifference( cur_lookAroundState.targetAngle, cur_state.yaw ) ) >= 5.0f / 180.0f * M_PI ){
                // continue turning
                // std::cout <<" Keep turning to that angle..." << std::endl;
                continue;
            }
            else{
                // Ready to go to path
                std::cout <<" Turn to the target angle, ready to go to it..." << std::endl;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
                cur_pathReachingState.pathReadyToGo = true;
                ori_front = front;
                ori_rear = rear;

                // Reset other flags
                cur_lookAroundState.turnStarted = false;
                cur_lookAroundState.clearPathCheckStarted = false;
            }
        }   
    }

    retCode = 0;
    return retCode;
}