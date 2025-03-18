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

    // Endless loop; end the program by pressing Ctrl-C.
    float safe_dist = 0.5f;
    float front_looking_dist = 0.5f;
    float dist_moved = 0.0f;
    float dist_moved_so_far = 0.0f;
    float angle_moved = 0.0f;
    int Timer = 0;
    int timer_max = 1000;
    bool Turning_mode = false;
    bool hasTakeoff = false;
    bool hasClearPath = false;
    bool is_Goto_started = false;
    bool is_looking_around_started = false;
    float current_front = 0.0f;
    float ori_angle = 0.0f;
    float start_turning_angle = 0.0f;
    float current_angle = 0.0f;
    float pre_angle = -5.0f;
    float current_angle_dev = 0.0f;
    int looking_around_timer = 0;
    bool hasFront = false;
    bool FoundFront = false;
    bool is_record_ori_angle = false;
    std::vector<std::pair<float, float>> angle_dev_vec;
    std::vector<std::pair<float, float>> angle_dev_candidate_vec;
    bool is_tg_reaching_turning_started = false;
    bool is_tg_reaching_goto_started = false;
    bool ready_to_reach_target = false;
    float battery_threshold = 3.15f;
    float angle_to_turn{-1.0f};
    float cur_path_front{0.0f};

    float cur_dodging_left{-1.0f};
    float cur_dodging_right{-1.0f};
    float tobackto_dodging_left{-1.0f};
    float tobackto_dodging_right{-1.0f};
    bool is_dodging_start = false;
    bool is_backto_start = false;
    bool test = false;
    int testTimer = 0;
    bool GOTO_MODE = false;
    bool GOTO_LDODGING_MODE = false;
    bool GOTO_RDODGING_MODE = false;
    bool DODGING_MODE = false;
    bool DODGING_LDODGING_MODE = false;
    bool DODGING_RDODGING_MODE = false;
    bool BACKTO_MODE = false;
    bool HOMING_MODE = false;
    bool REACHING_MODE = false;
    float dist_to_move = 0.0f;
    int16_t move_time = 5;
    int dodginTimer = 0;
    struct posState {
        float front;
        float left;
        float right;
    };
    std::vector<posState> pos_LRstate_vec;
    float start_front{-1.0f};
    float start_rear{-1.0f};
    bool is_chpad_reaching_turning_started = false;
    bool is_chpad_reaching_goto_started = false;
    bool ready_to_reach_chpad = false;
    float homing_angle{-4.0f};
    float homing_temp_angle{-4.0f};
    bool is_homing_turning_started = false;
    int timer_homing_turning = 0;
    bool is_backhome_turning_started = false;
    bool is_backhome_goto_started = false;
    while (od4.isRunning()) {
        // Sleep for 10 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Refresh pos state
        if ( pos_LRstate_vec.size() > 0 && start_front != -1.0f ){
            float cur_front_dist = std::abs(front - start_front);
            float cur_rear_dist = std::abs(rear - start_rear);
            for ( auto& state : pos_LRstate_vec ){
                if ( state.front >= 0.0f ){
                    if (  std::abs( state.front - cur_front_dist ) <= 0.01f ){
                        if ( state.left != -1.0f ){
                            state.left = left;
                        }
                        else if ( state.right != -1.0f ){
                            state.right = right;
                        }
                    }
                }                
                else{
                    if (  std::abs( - state.front - cur_rear_dist ) <= 0.01f ){
                        if ( state.left != -1.0f ){
                            state.left = left;
                        }
                        else if ( state.right != -1.0f ){
                            state.right = right;
                        }
                    }
                }
            }
        }

        /*
            Takeoff
            - Do it before start any goto action
        */
        if ( hasTakeoff == false && cur_state.battery_state > 3.8f ){
            Takeoff(od4, 1.0f, 3);
            hasTakeoff = true;
        }

        /*
            Obstacle avoidance
            - Check that whether any direction is too close
        */
        float safe_front_dist = 0.5f;
        float safe_left_dist = 0.1f;
        if ( GOTO_MODE ){
            if ( front <= safe_front_dist + dist_to_move / move_time + 0.1f ){
                std::cout <<" GOTO action meets limit." << std::endl;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);   // Stop flying in current direction

                // Homing mode
                if ( HOMING_MODE ){
                    homing_angle = wrap_angle( cur_state.yaw + M_PI );
                }

                GOTO_MODE = false;
                hasClearPath = false;
                pre_angle = cur_state.yaw + M_PI;
                continue;
            }
            else if ( left <= safe_left_dist ){
                std::cout <<" GOTO action close to left..." << std::endl;
                Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);   // Fly slightly to right
                GOTO_MODE = false;
                hasClearPath = true;
                pre_angle = cur_state.yaw + M_PI;
                continue;
            }
            else if ( right <= safe_left_dist ){
                std::cout <<" GOTO action close to right..." << std::endl;
                Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);   // Fly slightly to left
                GOTO_MODE = false;
                hasClearPath = true;
                pre_angle = cur_state.yaw + M_PI;
                continue;
            }
        }

        /*
            Dynamic obstacles dodging
            - Check that whether there are purple balls occur in the vision
            - Dodge the ball according to the ball's movement
        */
       if ( dist_obs <= 250.0f && dist_obs > -1.0f ){    // Means that some obstacles approach
            // std::cout <<" In dodging mode..." << std::endl;
            // dist_to_move = rear;
            // move_time = 5;
            float cur_front_dist = std::abs( front - start_front );
            float cur_rear_dist = std::abs( rear - start_rear );
            bool hasLeft = true;
            bool hasRight = true;
            if ( pos_LRstate_vec.size() <= 0 ){
                hasLeft = false;
                hasRight = false;
            }
            else{
                for ( const auto& state : pos_LRstate_vec ){
                    if ( state.front >= 0.0f ){
                        float front_dist_dev = std::abs( state.front  - cur_front_dist );
                        if ( front_dist_dev <= 0.1f ){
                            if ( state.left <= 0.3f ){
                                hasLeft = false;
                            }
                            else if ( state.right <= 0.3f ){
                                hasRight = false;
                            }
                        }
                    }
                    else{
                        float rear_dist_dev = std::abs( state.front  - cur_rear_dist );
                        if ( rear_dist_dev <= 0.1f ){
                            if ( state.left <= 0.3f ){
                                hasLeft = false;
                            }
                            else if ( state.right <= 0.3f ){
                                hasRight = false;
                            }
                        }
                    }
    
                    if ( hasLeft == false && hasRight == false ){
                        break;
                    }
                }
            }

            if ( aimDirection_obs >= 0.0f ){
                if ( hasLeft ){
                    std::cout <<" Try to dodge to the left..." << std::endl;
                    Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);
                    while ( dist_obs > -1.0f ){
                        // Wait until the obstacle leave
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    std::cout <<" Try to go back to the right..." << std::endl;
                    Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);
                }
                else{
                    if ( rear > 0.5f ){
                        std::cout <<" Try to dodge to the back..." << std::endl;
                        Goto(od4, - 0.5f * std::cos( current_angle ), - 0.5f * std::sin( current_angle ), 0.0f, 0.0f, 0);
                    }
                    else{
                        std::cout <<" Dodging reaching the wall, try to fly up..." << std::endl;
                        Goto(od4, 0.0f, 0.0f, 0.3f, 0.0f, 0, 1, true);
                        Goto(od4, 0.0f, 0.0f, -0.3f, 0.0f, 0, 1, true);
                    }
                }
            }
            else{
                if ( hasRight ){
                    std::cout <<" Try to dodge to the right..." << std::endl;
                    Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);
                    while ( dist_obs > -1.0f ){
                        // Wait until the obstacle leave
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    std::cout <<" Try to go back to the left..." << std::endl;
                    Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);
                }
                else{
                    if ( rear > 0.5f ){
                        std::cout <<" Try to dodge to the back..." << std::endl;
                        Goto(od4, - 0.2f * std::cos( current_angle ), - 0.2f * std::sin( current_angle ), 0.0f, 0.0f, 0, 1, true);
                    }
                    else{
                        std::cout <<" Dodging reaching the wall, try to fly up..." << std::endl;
                        Goto(od4, 0.0f, 0.0f, 0.3f, 0.0f, 0, 1, true);
                        Goto(od4, 0.0f, 0.0f, -0.3f, 0.0f, 0, 1, true);
                    }
                }
            }

            //Reset flag
            if ( hasClearPath ){
                hasClearPath = false;
            }
            
            if ( is_looking_around_started ){
                is_looking_around_started = false;
            }

            if ( FoundFront ){
                FoundFront = false;
            }
            continue;
        }
        
        /*
            Homing
            - Check the battery status
            - Go back to charging state while the status gets too low
        */   
        if ( cur_state.battery_state <= 4.2f && aimDirection_chpad > -4.0f && ready_to_reach_chpad == false ){
            std::cout <<" Found the chpad..." << std::endl;
            // Reset flag
            hasClearPath = false;
            is_Goto_started = false;
            hasFront = false;
            FoundFront = false;
            is_record_ori_angle = false;
            is_looking_around_started = false;

            if ( is_chpad_reaching_turning_started == false ){
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);
                ori_angle = cur_state.yaw; // Taking from communication
                float ang_turn = aimDirection_chpad + 20.0f / 180 * 3.14;
                if ( aimDirection_chpad < 0.0f ){
                    ang_turn = aimDirection_chpad - 20.0f / 180 * 3.14;
                } 
                Goto(od4, 0.0f, 0.0f, 0.0f, ang_turn, 10);
                is_chpad_reaching_turning_started = true;
                continue;
            }

            if ( std::abs( aimDirection_chpad )  <= 3.0f / 180 * M_PI ){
                if ( is_chpad_reaching_turning_started ){
                    is_chpad_reaching_turning_started = false;
                }

                ready_to_reach_chpad = true;
            }
            continue;
        }
        else if ( ready_to_reach_chpad ){
            if ( dist_chpad > -1.0f ){
                std::cout <<" Found the chpad, going to get closer" << std::endl;
                // Reset flag
                hasClearPath = false;
                is_Goto_started = false;
                hasFront = false;
                FoundFront = false;
                is_record_ori_angle = false;
                is_looking_around_started = false;
                
                if ( is_chpad_reaching_goto_started == false ){
                    current_angle = cur_state.yaw;
                    Goto(od4, 3.0f * std::cos( current_angle ), 3.0f * std::sin( current_angle ), 0.0f, 0.0f, 5);
                    GOTO_MODE = true;
                    is_chpad_reaching_goto_started = true;
                    continue;
                }
    
                if ( dist_chpad <= 80.0f ){
                    is_chpad_reaching_goto_started = false;    
                    ready_to_reach_chpad = false;
                    GOTO_MODE = false;
                    HOMING_MODE = true;
                    homing_angle = -4.0f;
                    std::cout <<" Reach the chpad..." << std::endl;
                    break; 
                }
                else{
                    continue;
                }
            }
        }
        else{
            ready_to_reach_chpad = false;
            is_chpad_reaching_turning_started = false;
            is_chpad_reaching_goto_started = false;
        }
        
        if ( cur_state.battery_state <= battery_threshold ){   // Take from communication or state estimator
            std::cout <<" Battery is too low..." << std::endl;
            // Do Go back motion
            
            // Implement landing for now
            if ( hasTakeoff ){
                Landing(od4, 0.0f, 3);
                Stopping(od4);
                hasTakeoff = false;
            }
            break;
        }

        /*
            Target reaching
            - Check that whether there has green ball as target
            - If green ball exist, then crazyflie will go to it straightforwardly
            - Turn off Goto_started and looking around started while some target occur
        */
        if ( aimDirection_target > -4.0f && ready_to_reach_target == false ){
            std::cout <<" Found the target." << std::endl;
            // Reset flag
            hasClearPath = false;
            is_Goto_started = false;
            hasFront = false;
            FoundFront = false;
            is_record_ori_angle = false;
            is_looking_around_started = false;
            is_homing_turning_started = false;
            if ( GOTO_MODE ){
                GOTO_MODE = false;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);
            }

            if ( is_tg_reaching_turning_started == false ){
                ori_angle = cur_state.yaw; // Taking from communication
                float ang_turn = aimDirection_target + 20.0f / 180 * 3.14;
                if ( aimDirection_target < 0.0f ){
                    ang_turn = aimDirection_target - 20.0f / 180 * 3.14;
                } 
                Goto(od4, 0.0f, 0.0f, 0.0f, ang_turn, 10);
                is_tg_reaching_turning_started = true;
                continue;
            }

            if ( std::abs( aimDirection_target )  <= 3.0f / 180 * M_PI ){
                if ( is_tg_reaching_turning_started ){
                    is_tg_reaching_turning_started = false;
                }

                ready_to_reach_target = true;
            }
            continue;
        }
        else if ( ready_to_reach_target ){
            if ( dist_target > -1.0f ){
                std::cout <<" Found the target, going to get closer" << std::endl;
                // Reset flag
                hasClearPath = false;
                is_Goto_started = false;
                hasFront = false;
                FoundFront = false;
                is_record_ori_angle = false;
                is_looking_around_started = false;
                is_homing_turning_started = false;
                
                if ( is_tg_reaching_goto_started == false ){
                    current_angle = cur_state.yaw;
                    Goto(od4, 3.0f * std::cos( current_angle ), 3.0f * std::sin( current_angle ), 0.0f, 0.0f, 5);
                    is_tg_reaching_goto_started = true;
                    continue;
                }
    
                if ( dist_target <= 80.0f ){
                    is_tg_reaching_goto_started = false;    
                    ready_to_reach_target = false;
                    std::cout <<" Reach the target, land and stop." << std::endl;
                    Landing(od4, 0.0f, 3);
                    Stopping(od4);
                    break; 
                }
                else{
                    continue;
                }
            }
        }
        else{
            if ( HOMING_MODE && homing_angle != -4.0f ){
                std::cout <<" Now the battery is low, start to look around left and right to find the target" << std::endl;
                if ( is_homing_turning_started == false ){
                    start_turning_angle = cur_state.yaw;
                    if ( timer_homing_turning == 0 ){
                        Goto(od4, 0.0f, 0.0f, 0.0f, 120.0f / 180 * M_PI, 5);
                    }
                    else if ( timer_homing_turning >= 1 ){
                        Goto(od4, 0.0f, 0.0f, 0.0f, -120.0f / 180 * M_PI, 5);
                    }
                    is_homing_turning_started = true;
                }
                else{
                    if ( std::abs( angleDifference( start_turning_angle, cur_state.yaw ) ) >= 120.0f / 180 * M_PI - 1 / 180.0f * M_PI ){
                        timer_homing_turning += 1;
                        is_homing_turning_started = false;
                        if ( timer_homing_turning > 2 ){
                            std::cout <<" No target, go back home..." << std::endl;
                            if ( is_backhome_turning_started == false ){
                                Goto(od4, 0.0f, 0.0f, 0.0f, wrap_angle(cur_state.yaw - homing_angle), 1);
                                is_backhome_turning_started = true;
                            }
                            else if ( std::abs( angleDifference( homing_angle, cur_state.yaw ) ) <= 5.0f / 180 * M_PI ){
                                is_backhome_turning_started = false;
                                if ( is_backhome_goto_started == false ){
                                    Goto(od4, 0.0f, 0.0f, 0.0f, wrap_angle(cur_state.yaw - homing_angle), 3); 
                                    is_backhome_goto_started = true;
                                }                               
                            }
                        }
                    }
                }                
            }
            else{
                is_tg_reaching_turning_started = false;
                is_tg_reaching_goto_started = false;
                ready_to_reach_target = false;
                is_homing_turning_started = false;
                timer_homing_turning = 0;
                is_backhome_turning_started = false;
                is_backhome_goto_started = false;
            }
        }

        /*
            Front reaching
            - If path exist, move forward until the front dist less than some threshold
            - After reaching it, do look around action to find target
        */
        // float move_dist = cur_path_front;
        // int gotoTime = 5;
        if ( hasClearPath ){
            if ( GOTO_MODE == false ){
                std::cout <<" Found a clear path start GOTO action." << std::endl;
                dist_to_move = front;
                move_time = 5;
                current_angle = cur_state.yaw;
                Goto(od4, dist_to_move * std::cos( current_angle ), dist_to_move * std::sin( current_angle ), 0.0f, 0.0f, move_time);
                GOTO_MODE = true;
            }
            continue; 
        }

        /*
            Look around
            - Turn around for 360 degree to see whether target exist
            - In the mean time, record some possible front path to go to
        */
       if ( hasFront == false ){
            if ( is_tg_reaching_turning_started ){
                is_tg_reaching_turning_started = false;
            }

            if ( is_record_ori_angle == false ){
                std::cout <<" No front, start record the original angle." << std::endl;
                // ori_angle = cur_state.yaw; // Taking from communication
                angle_dev_vec.clear();
                angle_dev_candidate_vec.clear();
                is_record_ori_angle = true;

                current_angle = cur_state.yaw; // Taking from communication
                std::cout <<" No front, The original yaw recorded: " << current_angle << std::endl;
            }

            if ( is_looking_around_started == false ){
                std::cout <<" No front, start looking around." << std::endl;
                start_turning_angle = cur_state.yaw;
                Goto(od4, 0.0f, 0.0f, 0.0f, M_PI / 2.0f + 10 / 180.0f * M_PI, 3);
                is_looking_around_started = true;
            }

            current_angle = cur_state.yaw; // Taking from communication
            angle_dev_vec.insert(angle_dev_vec.begin(),{ front, current_angle });
            angle_dev_vec.insert(angle_dev_vec.begin(),{ rear, wrap_angle(current_angle + M_PI) });
            angle_dev_vec.insert(angle_dev_vec.begin(),{ left, wrap_angle(current_angle + M_PI / 2.0f) });
            angle_dev_vec.insert(angle_dev_vec.begin(),{ right, wrap_angle(current_angle - M_PI / 2.0f) });
            if ( front >= front_looking_dist ){
                // std::cout <<" No front, has candidate direction with front:" << front << " , and angle:" << current_angle << std::endl;
                angle_dev_candidate_vec.insert(angle_dev_candidate_vec.begin(),{front,current_angle});
            }
            if ( rear >= front_looking_dist ){
                // std::cout <<" No front, has candidate direction with rear:" << rear << " , and angle:" << wrap_angle(current_angle + M_PI) << std::endl;
                angle_dev_candidate_vec.insert(angle_dev_candidate_vec.begin(),{ rear, wrap_angle(current_angle + M_PI) });
            }
            if ( left >= front_looking_dist ){
                // std::cout <<" No front, has candidate direction with left:" << left << " , and angle:" << wrap_angle(current_angle + M_PI / 2.0f) << std::endl;
                angle_dev_candidate_vec.insert(angle_dev_candidate_vec.begin(),{ left, wrap_angle(current_angle + M_PI / 2.0f) });
            }
            if ( right >= front_looking_dist ){
                // std::cout <<" No front, has candidate direction with right:" << right << " , and angle:" << wrap_angle(current_angle - M_PI / 2.0f) << std::endl;
                angle_dev_candidate_vec.insert(angle_dev_candidate_vec.begin(),{ right, wrap_angle(current_angle - M_PI / 2.0f) });
            }

            // Check whether to end the looking around motion
            if ( std::abs( angleDifference( start_turning_angle, current_angle ) ) >= M_PI / 2.0f - 1 / 180.0f * M_PI ){
                std::cout <<" No front, Complete quater circle looking. Current yaw is: " << current_angle << std::endl;
                Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
                
                // Here means we found some candidates
                std::cout <<" No front, Found cadidate direction with size:" << angle_dev_candidate_vec.size() << std::endl;
                // std::sort(angle_dev_vec.begin(), angle_dev_vec.end(), [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                //     if(a.first != b.first)
                //         return a.first > b.first;
                // });
                std::sort(angle_dev_candidate_vec.begin(), angle_dev_candidate_vec.end(), [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                    if(a.first != b.first)
                        return a.first > b.first;
                });

                ori_angle = cur_state.yaw; // Taking from communication
                hasFront = true;
                is_record_ori_angle = false;
                is_looking_around_started = false;
            }   
            else{
                // std::cout <<" No front, keep turning..." << std::endl;
                continue;
            }         
        }

        /*
            Front checking
            - Check that whether there has clear path in front of current direction
            - If no target and currnet fron has no path, move to the next longest (perhaps avoiding past) front to check
        */
        bool hasObOnPath = false; 
        if ( hasFront ){
            // std::cout <<" Has front, start checking." << std::endl;
            if ( FoundFront == false ){
                ori_angle = cur_state.yaw; // Taking from communication
                std::cout <<" Has front, start looping with candidate size:" << angle_dev_candidate_vec.size() << ", and overall vector size:" << angle_dev_vec.size() << std::endl;
                for(const auto& pair_cand : angle_dev_candidate_vec) {
                    // std::cout <<" current candidate front:" << pair_cand.first << ", angle:" << pair_cand.second << std::endl;
                    if ( pre_angle != -5.0f && std::abs( angleDifference( pre_angle, pair_cand.second ) ) <= 10.0f / 180 * M_PI ){
                        // std::cout <<"     Angle diff smaller than 10.0f..." << std::endl;
                        continue;
                    }

                    float angMin = std::abs( std::atan2( 0.1f, pair_cand.first ) );
                    hasObOnPath = false;
                    // std::cout <<" Start check path, current front:" << pair_cand.first << ", angle:" << pair_cand.second << ", angle min: " << angMin << std::endl;
                    for(const auto& pair : angle_dev_vec) {
                        float angDev = std::abs( angleDifference( pair.second, pair_cand.second ) );
                        if ( angDev <= angMin ){
                            if ( pair.first * std::cos( angDev ) < pair_cand.first - 0.5f ){
                                std::cout <<"     Path has obs to lower degree, current angle:" << pair.second << ", current front:" << pair.first << ", angle dev: " << angDev << ", in path length: " << pair.first * std::sin( angDev ) << std::endl;
                                hasObOnPath = true;
                                break;
                            }
                        }
                        else if ( angDev <= 45.0f / 180.0f * M_PI && angDev > angMin ){
                            if ( pair.first * std::sin( angDev ) <= 0.1f ){
                                std::cout <<"     Path has obs to 45 degree, current angle:" << pair.second << ", current front:" << pair.first << ", angle dev: " << angDev << ", in path length: " << pair.first * std::sin( angDev ) << std::endl;
                                hasObOnPath = true;
                                break;
                            }
                        }
                        // std::cout <<"     Path has no obs or larger than 45 degree with:" << angDev << " degree dev and front of " << pair.first << std::endl;
                    }

                    if ( hasObOnPath == false ){
                        FoundFront = true;
                        angle_to_turn = angleDifference( ori_angle, pair_cand.second );
                        // cur_path_front = pair_cand.first;
                        float angTurn = angle_to_turn + 5 / 180.0f * M_PI;
                        if ( angle_to_turn < 0.0f ){
                            angTurn = angle_to_turn - 5 / 180.0f * M_PI;
                        }

                        // Record current LRstate
                        pos_LRstate_vec.clear();
                        for (const auto& pair : angle_dev_vec){
                            // float angDev = wrap_angle(angleDifference( pair_cand.second, pair.second ));
                            float angDev = wrap_angle(angleDifference( M_PI / 2.0f, pair.second ));    // For test
                            posState state;
                            state.front = pair.first * std::cos( angDev );
                            if ( pair.first * std::sin( angDev ) > 0.0f ){
                                state.left = pair.first * std::sin( angDev );
                                state.right = -1.0f;
                            }
                            else{
                                state.right = std::abs(pair.first * std::sin( angDev ));
                                state.left = -1.0f;
                            }
                            pos_LRstate_vec.insert(pos_LRstate_vec.begin(),state);
                        }
                        start_front = front;
                        start_rear = rear;

                        Goto(od4, 0.0f, 0.0f, 0.0f, wrap_angle( angTurn ), 3);
                        std::cout <<" Found a direction to go to, start turning to angle: " << pair_cand.second << " with ang dev: " << angle_to_turn << std::endl;
                        break;
                    }
                }

                

                if ( FoundFront == false ){
                    FoundFront = true;
                    angle_to_turn = angleDifference( ori_angle, pre_angle );
                    // cur_path_front = 1.5f;
                    float angTurn = angle_to_turn + 5 / 180.0f * M_PI;
                    if ( angle_to_turn < 0.0f ){
                        angTurn = angle_to_turn - 5 / 180.0f * M_PI;
                    }

                    // Record current LRstate
                    pos_LRstate_vec.clear();
                    for (const auto& pair : angle_dev_vec){
                        float angDev = wrap_angle(angleDifference( pre_angle, pair.second ));
                        posState state;
                        state.front = pair.first * std::cos( angDev );
                        if ( pair.first * std::sin( angDev ) > 0.0f ){
                            state.left = pair.first * std::sin( angDev );
                            state.right = -1.0f;
                        }
                        else{
                            state.right = std::abs(pair.first * std::sin( angDev ));
                            state.left = -1.0f;
                        }
                        pos_LRstate_vec.insert(pos_LRstate_vec.begin(),state);
                    }
                    start_front = front;
                    start_rear = rear;

                    Goto(od4, 0.0f, 0.0f, 0.0f, wrap_angle( angTurn ), 3);
                    std::cout <<" No direction to go to, start turning to the original angle: " << pre_angle << " with ang dev: " << angle_to_turn << std::endl;                
                }
            }
            else{
                current_angle = cur_state.yaw; // Taking from communication
                // std::cout <<" Found front, still turning to the angle to turn:" << angle_to_turn << ", original angle: " << ori_angle << ", current angle: " << current_angle << std::endl;
                if ( std::abs( angleDifference( ori_angle + angle_to_turn, current_angle ) ) <= 5.0f / 180.0f * M_PI ){
                    std::cout <<" Found a direction to go to, turning ended, about to do goto action." << std::endl;
                    Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
                    cur_path_front = front;
                    hasFront = false;
                    FoundFront = false;
                    hasClearPath = true;
                    angle_to_turn = -1.0f;
                }
            }
        }
    }

    retCode = 0;
    return retCode;
}