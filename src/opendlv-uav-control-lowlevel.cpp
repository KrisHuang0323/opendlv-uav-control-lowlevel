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
 
 
 void Takeoff(cluon::OD4Session &od4, float height, int duration){
     std::cout << "Taking off to height: " << height << std::endl;        
     cluon::data::TimeStamp sampleTime;
     opendlv::logic::action::CrazyFlieCommand cfcommand;
     cfcommand.height(height);
     cfcommand.time(duration);
     od4.send(cfcommand, sampleTime, 0);
     std::this_thread::sleep_for(std::chrono::milliseconds(duration*1000 + 500));
 }
 
 void Goto(cluon::OD4Session &od4, float x, float y, float z, float yaw, int duration, int relative = 1, bool isdelay = false, bool verbose = true){
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
     float takeoff_batterythreshold = 3.6f;
 
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
     float homing_batterythreshold = 3.35f;
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
                 taskStartTime = std::chrono::high_resolution_clock::now();
             }
             else{
                 std::cout <<" Battery is too low for taking off..." << std::endl;
                 continue;
             }
         }
 
         /*
             Valid direction check
             - Check from the range recording array
         */
         // Recheck current dist path vector if in turning mode
         if ( on_TURNING_MODE && angleFrontState_vec.size() > 0 ){
             // Try to refresh the rear distance
             float rearDist{-1.0f};
             float angMin = std::abs( std::atan2( 0.1f, front ) );
             for ( const auto& pair : angleFrontState_vec ){
                 // std::cout <<" Current angle to check, angle: " << pair.angle << ", front: " << pair.front << std::endl; 
                 float angDev = std::abs( angleDifference( pair.angle, cur_state.yaw ) );
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
             if ( distPathstate_vec.size() > 0 ){
                 distPathstate_vec.clear();
             }
             for ( const auto& pair : angleFrontState_vec ){
                 float angDev = angleDifference( cur_state.yaw, pair.angle );
                 // std::cout <<" Current angle difference: " << angDev << ", front: " <<pair.front << ", current angle: " << cur_state.yaw << std::endl; 
                 // std::cout <<" Cosine(On Path): " << pair.front*std::cos( angDev ) << ", Sine(Dev Path): " << pair.front*std::sin( angDev ) << std::endl; 
                 distPathState pstate = { pair.front*std::cos( angDev ), pair.front*std::sin( angDev ) }; 
                 distPathstate_vec.insert( distPathstate_vec.begin(), pstate );              
             }
         }
 
         // Refresh valid left/right/rear on the way
         if ( distPathstate_vec.size() > 0 && on_GoTO_MODE ){
             float cur_dist_onPath = front - ori_front;
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
 
             // Refresh rear
             cur_validWay.toRear = rear;
             // std::cout <<" Refresh distpath on goto mode with rear: " << cur_validWay.toRear << std::endl;
         }
 
         // Start valid way checking
         if ( distPathstate_vec.size() > 0 ){
             cur_validWay.toLeft = -1.0f;
             cur_validWay.toRight = -1.0f;
 
             float cur_dist_onPath = front - ori_front;
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
             // std::cout <<" Valid way checking start... with left " << cur_validWay.toLeft << ", with right: " << cur_validWay.toRight << ", with rear: " << cur_validWay.toRear << ", and current angle: " << cur_state.yaw << std::endl;
         }
 
         /*
             Obstacle Avoidance for walls or static obstacles
             - Get the range from rangefinder
             - Check whether some ranges reach ends
         */
         float safe_dist{0.0f};
         if ( front >= 1.0f )
             safe_dist = safe_endreach_dist + front / 3 + 0.35f;
         else
             safe_dist = safe_endreach_dist;
         if ( front <= safe_dist ){
             obsStaticStartTime = std::chrono::high_resolution_clock::now();
             if ( on_GoTO_MODE ){
                 std::cout <<" Front end meets limit." << std::endl;
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                 cur_reachEndState.reachFront = true; 
                 on_GoTO_MODE = false;
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
             }           
         }
         else{
             cur_reachEndState.reachFront = false;
         }
 
         if ( left <= safe_endreach_LR_dist ){
             if ( on_TURNING_MODE ){
                 cur_preDist.left = -1.0f;
             }
             else{
                 if ( cur_preDist.left == -1.0f || cur_preDist.left <= left ){
                     cur_preDist.left = left;
                     std::cout <<" Record left: "<< cur_preDist.left << std::endl;
                     obsStaticStartTime = std::chrono::high_resolution_clock::now();
                     continue;
                 }
                 else{
                     std::cout <<" Cur left: "<< left << std::endl;
                     std::cout <<" Cur toRight: "<< cur_validWay.toRight << std::endl;
                     if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f && dist_obs == -1.0f ){
                         std::cout <<" Left end meets limit with right direction dodge..." << std::endl;
                         Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                         has_possibleInterrupt = true;
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
                         continue;
                     }
                     else{
                         std::cout <<" Left end meets limit." << std::endl;
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                         cur_reachEndState.reachLeft = true;
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
                     }
                 }                
             }
         }
         else{ 
             cur_reachEndState.reachLeft = false;
         }
 
         if ( right <= safe_endreach_LR_dist ){
             if ( on_TURNING_MODE ){
                 cur_preDist.right = -1.0f;
             }
             else{
                 if ( cur_preDist.right == -1.0f || cur_preDist.right <= right ){
                     cur_preDist.right = right;
                     std::cout <<" Record right: "<< cur_preDist.right << std::endl;
                     obsStaticStartTime = std::chrono::high_resolution_clock::now();
                     continue;
                 }
                 else{
                     std::cout <<" Cur right: "<< right << std::endl;
                     std::cout <<" Cur toLeft: "<< cur_validWay.toLeft << std::endl;
                     if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f && dist_obs == -1.0f ){
                         std::cout <<" Right end meets limit with left direction dodge..." << std::endl;
                         Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                         has_possibleInterrupt = true;
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
                         continue;
                     }
                     else{
                         std::cout <<" Right end meets limit." << std::endl;
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction  
                         cur_reachEndState.reachRight = true;
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
                     }
                 }
             }
         }
         else{
             cur_reachEndState.reachRight = false;
         }
 
         if ( rear <= safe_endreach_dist ){
             obsStaticStartTime = std::chrono::high_resolution_clock::now();
             if ( cur_dodgeType != DODGE_NONE ){
                 std::cout <<" Rear end meets limit." << std::endl;
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);   // Stop flying in current direction
                 cur_reachEndState.reachRear = true;
                 on_GoTO_MODE = false;
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
         if ( dist_obs > -1.0f ){    // Means that some obstacles approach
             std::cout <<" Has dynamic obstacles with to left: " << cur_validWay.toLeft << ", to right: " << cur_validWay.toRight << ", to rear: " << cur_validWay.toRear << ", obs on path: " << dist_obs * std::cos( aimDirection_obs ) << ", dist: " << dist_obs << ", aimDirection: " << aimDirection_obs << std::endl; 
             obsDynamicStartTime = std::chrono::high_resolution_clock::now();
 
             if ( dist_obs * std::cos( aimDirection_obs ) <= 135.0f ){    
                 std::cout <<" Obstacle gets too close..." << std::endl;            
                 if ( aimDirection_obs < 0.0f ){
                     // If left side has valid way, dodge to it
                     if ( cur_validWay.toLeft >= safe_endreach_LR_dist + 0.2f ){
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                         std::cout <<" Try to dodge to the left..." << std::endl;
                         Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                         dodgeDist += 0.2f;
                         on_GoTO_MODE = false;
                         cur_dodgeType = DODGE_LEFT;
                         continue;
                     }
                     else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                         std::cout <<" Try to dodge to the rear..." << std::endl;
                         cur_distToMove = cur_validWay.toRear;
                         if ( cur_distToMove >= 1.0f )
                             time_toMove = 3;
                         else
                             time_toMove = 5;
                         Goto(od4, - cur_distToMove * std::cos( cur_state.yaw ), - cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                         on_GoTO_MODE = true;
                         cur_dodgeType = DODGE_REAR;
                         has_dodgeToRear = true;
                         continue;
                     }
                     else{
                         std::cout <<" Can not dodge..." << std::endl;
                     }
                 }
                 else{
                     // If right side has valid way, dodge to it
                     if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f ){
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                         std::cout <<" Try to dodge to the right..." << std::endl;
                         Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                         dodgeDist -= 0.2f;
                         on_GoTO_MODE = false;
                         cur_dodgeType = DODGE_RIGHT;
                         continue;
                     }
                     else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop first
                         std::cout <<" Try to dodge to the rear..." << std::endl;
                         cur_distToMove = cur_validWay.toRear;
                         if ( cur_distToMove >= 1.0f )
                             time_toMove = 3;
                         else
                             time_toMove = 5;
                         Goto(od4, - cur_distToMove * std::cos( cur_state.yaw ), - cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                         on_GoTO_MODE = true;
                         cur_dodgeType = DODGE_REAR;
                         has_dodgeToRear = true;
                         continue;
                     }
                     else{
                         std::cout <<" Can not dodge..." << std::endl;
                     }
                 }
             }
 
             if ( cur_dodgeType == DODGE_NONE ){
                 // Stop first and record current dist_obs
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true);
                 cur_obsState.dist_obs = dist_obs;
                 cur_obsState.aimDirection_obs = aimDirection_obs;
                 cur_dodgeType = DODGE_STOP;
                 continue;
             }
 
             // Check if the obstacle is on the left or right side            
             enum localDodgeType {
                 LOCAL_DODGE_LEFT,
                 LOCAL_DODGE_RIGHT
             };
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
                         Goto(od4, - 0.2f * std::sin( cur_state.yaw ), 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying left to dodge
                         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                         dodgeDist += 0.2f;
                         on_GoTO_MODE = false;
                         cur_dodgeType = DODGE_LEFT;
                         continue;
                     }
                     else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                         // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                         std::cout <<" Try to dodge to the rear..." << std::endl;
                         cur_distToMove = cur_validWay.toRear;
                         if ( cur_distToMove >= 1.0f )
                             time_toMove = 3;
                         else
                             time_toMove = 5;
                         Goto(od4, - cur_distToMove * std::cos( cur_state.yaw ), - cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                         on_GoTO_MODE = true;
                         cur_dodgeType = DODGE_REAR;
                         has_dodgeToRear = true;
                         continue;
                     }
                     else{
                         std::cout <<" Can not dodge..." << std::endl;
                     }
                 }
                 else{
                     // If right side has valid way, dodge to it
                     if ( cur_validWay.toRight >= safe_endreach_LR_dist + 0.2f ){
                         // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                         std::cout <<" Try to dodge to the right..." << std::endl;
                         Goto(od4, 0.2f * std::sin( cur_state.yaw ), - 0.2f * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);    // Flying right to dodge
                         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                         dodgeDist -= 0.2f;
                         on_GoTO_MODE = false;
                         cur_dodgeType = DODGE_RIGHT;
                         continue;
                     }
                     else if ( cur_validWay.toRear >= safe_endreach_LR_dist + 0.2f && has_dodgeToRear == false ){
                         // Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, true); // Stop first
                         std::cout <<" Try to dodge to the rear..." << std::endl;
                         cur_distToMove = cur_validWay.toRear;
                         if ( cur_distToMove >= 1.0f )
                             time_toMove = 3;
                         else
                             time_toMove = 5;
                         Goto(od4, - cur_distToMove * std::cos( cur_state.yaw ), - cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);    // Flying right to dodge
                         on_GoTO_MODE = true;
                         cur_dodgeType = DODGE_REAR;
                         has_dodgeToRear = true;
                         has_InterruptNeedToReDo = true;
                         continue;
                     }
                     else{
                         std::cout <<" Can not dodge..." << std::endl;
                     }
                 }
             }
 
             // Record current dist and aimDirection for later use
             cur_obsState.dist_obs = dist_obs;
             cur_obsState.aimDirection_obs = aimDirection_obs;
         }
         else if ( cur_dodgeType != DODGE_NONE ){
             // Go back to the original position while the obstacle is gone
             // if ( dodgeDist_UP != 0.0f ){
             //     std::cout <<" No obs, try to fly down..." << std::endl;
             //     Goto(od4, 0.0f, 0.0f, dodgeDist_UP, 0.0f, 0, 1, true);
             // }
             if ( dodgeDist != 0.0f ){
                 std::cout <<" No obs, try to fly back..." << std::endl;
                 Goto(od4, dodgeDist * std::sin( cur_state.yaw ), - dodgeDist * std::cos( cur_state.yaw ), 0.0f, 0.0f, 0, 1, true);
             }
 
             // Reset flags            
             // dodgeDist_UP = 0.0f;
             dodgeDist = 0.0f;
             cur_dodgeType = DODGE_NONE; 
             has_dodgeToRear = false;
             on_GoTO_MODE = false;
             if ( has_InterruptNeedToReDo == false ){
                 has_possibleInterrupt = true;
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
         }        
         
         /*
             Target finding
             - Check that whether there has green ball or chpad as target
             - If green ball exist, check that the crazyflie has a clear path to it
             - Use the dist/aimDirection from last section
         */
         float dist_to_reach = dist_target;
         float aimDirection_to_reach = aimDirection_target;
         if ( cur_state.battery_state <= homing_batterythreshold ){
             dist_to_reach = dist_chpad;
             aimDirection_to_reach = aimDirection_chpad;
 
             // Find the charging pad, stop and do landing
             if ( dist_to_reach * std::cos( aimDirection_to_reach ) <= 30.0f && dist_to_reach != -1.0f ){
                 Landing(od4, 0.0f, 3);
                 Stopping(od4);
                 std::cout <<" Successfully do landing and stopping..." << std::endl;
 
                 // Record the end time
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
                 std::cout <<" , elapsed: " << elapsed.count() << " seconds(s)" << std::endl;
                 break;
             }
         }
 
         // If Interrupted
         if ( has_InterruptNeedToReDo ){
             if ( cur_targetCheckState.pointToTarget == false && ( cur_targetCheckState.turnStarted || cur_targetCheckState.aimTurnStarted ) ){
                 std::cout <<" Possible interruption without target..." << std::endl;
                 cur_targetCheckState.aimTurnStarted = false;
                 cur_targetCheckState.turnStarted = false;
                 has_InterruptNeedToReDo = false;
                 on_TURNING_MODE = false;
 
                 auto start_time_t = std::chrono::system_clock::to_time_t(
                     std::chrono::time_point_cast<std::chrono::system_clock::duration>(targetFindingStartTime)
                 );
                 const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - targetFindingStartTime;
                 std::cout <<" Original target finding start time: " << std::ctime(&start_time_t) << std::endl;
                 std::cout <<" Interrupted time is: " << elapsed.count() << "seconds(s)" << std::endl;
                 targetFindingStartTime = std::chrono::high_resolution_clock::now();
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
                 const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - lookAroundStartTime;
                 std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
                 std::cout <<" Interrupted time is: " << elapsed.count() << "seconds(s)" << std::endl;
                 lookAroundStartTime = std::chrono::high_resolution_clock::now();
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
                 const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - frontReachingStartTime;
                 std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
                 std::cout <<" Interrupted time is: " << elapsed.count() << "seconds(s)" << std::endl;
                 frontReachingStartTime = std::chrono::high_resolution_clock::now();
             }
         }
 
         // Try to point the crazyflie to the target first
         // std::cout <<" Current angle: " << aimDirection_to_reach << ", current battery state: "<< cur_state.battery_state << std::endl;  
         if ( cur_targetCheckState.pointToTarget == false && ( ( aimDirection_to_reach != -4.0f && dist_to_reach * std::cos( aimDirection_to_reach ) > 20.0f ) || cur_targetCheckState.aimTurnStarted ) ){
             if ( cur_targetCheckState.aimTurnStarted == false ){
                 targetFindingStartTime = std::chrono::high_resolution_clock::now();
                 std::cout <<" Find target start turning..." << std::endl;   
                 // Stop first
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false, false);
 
                 if ( aimDirection_to_reach > 0.0f ){    
                     float angTurn = 90.0f / 180.0f * M_PI;
                     cur_targetCheckState.ang_toTurn = angTurn;
                     cur_targetCheckState.startAngle = cur_state.yaw;    
                     Goto(od4, 0.0f, 0.0f, 0.0f, angTurn + 10.0f / 180.0f * M_PI, 2); 
                 }
                 else{
                     float angTurn = -90.0f / 180.0f * M_PI;
                     cur_targetCheckState.ang_toTurn = angTurn;
                     cur_targetCheckState.startAngle = cur_state.yaw;    
                     Goto(od4, 0.0f, 0.0f, 0.0f, angTurn - 10.0f / 180.0f * M_PI, 2);
                 }
                 std::cout <<" Angle to turn: " << cur_targetCheckState.ang_toTurn << std::endl;   
                 cur_targetCheckState.aimTurnStarted = true;    
                 on_TURNING_MODE = true;
                 continue;
             }
             else if ( cur_targetCheckState.turnStarted == false ){
                 if ( std::abs( angleDifference( cur_targetCheckState.startAngle, cur_state.yaw ) ) < std::abs(cur_targetCheckState.ang_toTurn) ){
                     // Do the returning if something interrupt
                     if ( has_possibleInterrupt ){
                         std::cout <<" Some targets occur, so try to turn to look around the target again..." << std::endl;
                         float angDev = std::abs( angleDifference( cur_targetCheckState.startAngle, cur_state.yaw ) );
                         if ( cur_targetCheckState.ang_toTurn > 0.0f ){
                             float angTurn = 90.0f / 180.0f * M_PI - angDev;
                             cur_targetCheckState.ang_toTurn = angTurn;
                             cur_targetCheckState.startAngle = cur_state.yaw;    
                             Goto(od4, 0.0f, 0.0f, 0.0f, angTurn + 10.0f / 180.0f * M_PI, 2);                             
                         }
                         else{
                             float angTurn = - ( 90.0f / 180.0f * M_PI - angDev );
                             cur_targetCheckState.ang_toTurn = angTurn;
                             cur_targetCheckState.startAngle = cur_state.yaw;    
                             Goto(od4, 0.0f, 0.0f, 0.0f, angTurn - 10.0f / 180.0f * M_PI, 2);          
                         } 
                         on_TURNING_MODE = true;
                         has_possibleInterrupt = false;
                         continue;
                     }
 
                     // Record the angle and front                    
                     angleFrontState state;
                     state.front = front;
                     state.angle = wrap_angle(cur_state.yaw);
                     angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                     
                     state.front = rear;
                     state.angle = wrap_angle(cur_state.yaw + M_PI);
                     angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
     
                     state.front = left;
                     state.angle = wrap_angle(cur_state.yaw + M_PI / 2.0f);
                     angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
     
                     state.front = right;
                     state.angle = wrap_angle(cur_state.yaw - M_PI / 2.0f);
                     angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
     
                     // Record target angle
                     // std::cout <<" Current aim direction diff:" << std::abs( aimDirection_to_reach ) << std::endl;   
                     if ( cur_targetCheckState.cur_aimDiff > std::abs( aimDirection_to_reach ) ){
                         cur_targetCheckState.targetAngle = cur_state.yaw;
                         cur_targetCheckState.cur_aimDiff = std::abs( aimDirection_to_reach );
                         std::cout <<" Current angle: "<< cur_state.yaw << " and aim diff: " << std::abs( aimDirection_to_reach ) << std::endl;   
                     }
                     continue;
                 }
                 else{
                     // Check the target angle to go to ( the clear path)
                     // Stop the turning action
                     std::cout <<" Complete turning and start to check..." << std::endl;   
                     Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                     std::this_thread::sleep_for(std::chrono::milliseconds(500));
         
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
                             std::cout <<" Current angle: " << cur_state.yaw << std::endl;  
                             std::cout <<" Target angle: " << cur_targetCheckState.targetAngle << std::endl; 
                             std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) << std::endl; 
                             float angTurn = angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) + 5.0f / 180.0f * M_PI;
                             if ( angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) < 0.0f ){
                                 angTurn -= 10.0f / 180.0f * M_PI;
                             }
                             Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1);
                             cur_targetCheckState.turnStarted = true;
                             break;
                         }
                     }
                     continue;
                 }
             }
             else{
                 if ( std::abs( angleDifference( cur_targetCheckState.targetAngle, cur_state.yaw ) ) >= 5.0f / 180.0f * M_PI ){
                     // Do the returning if something interrupt
                     if ( has_possibleInterrupt ){
                         std::cout <<" Some targets occur, so try to turn to the target again..." << std::endl;
                         float angTurn = angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) + 5.0f / 180.0f * M_PI;
                         if ( angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) < 0.0f ){
                             angTurn -= 10.0f / 180.0f * M_PI;
                         }
                         Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1); 
                         on_TURNING_MODE = true;
                         has_possibleInterrupt = false;
                         continue;
                     }
                     
                     // continue turning
                     // std::cout <<" Keep turning with angle diff: " << angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) << std::endl; 
                     continue;
                 }
                 else{ 
                     // Stop first
                     Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
 
                     // If current front is not goable
                     if ( front <= safe_endreach_dist + 0.7 / 2 ){
                         std::cout <<" Turn to the target angle, But current angle is not goable..." << std::endl;
                         angleFrontState state;
                         state.front = front;
                         state.angle = wrap_angle(cur_state.yaw);
                         angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
 
                         // Start to find another way to go to
                         // Check for clear path
                         bool hasObOnPath = false; 
                         for ( const auto& pair_cand : angleFrontState_vec ){
                             if ( pair_cand.front <= safe_endreach_dist){
                                 continue;
                             }
 
                             if ( std::abs( angleDifference( cur_state.yaw, pair_cand.angle ) ) <= 1.0 / 180.0f * M_PI ){
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
                                 std::cout <<" Current angle: " << cur_state.yaw << std::endl;  
                                 std::cout <<" Target angle: " << cur_targetCheckState.targetAngle << std::endl; 
                                 std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) << std::endl; 
                                 float angTurn = angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) + 5.0f / 180.0f * M_PI;
                                 if ( angleDifference( cur_state.yaw, cur_targetCheckState.targetAngle ) < 0.0f ){
                                     angTurn -= 10.0f / 180.0f * M_PI;
                                 }
                                 Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1);
                                 break;
                             }
                         }
                         continue;                        
                     }
 
                     // Ready to go to path
                     std::cout <<" Turn to the target angle, ready to go to it..." << std::endl;
                     cur_pathReachingState.pathReadyToGo = true;
                     cur_targetCheckState.pointToTarget = true;
                     ori_front = front;
     
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
                 }
                 continue;
             }
         }
 
         /*
             Front reaching
             - Go to the path straightforwardly if permitted
         */
         // Switch between target and charging pad according to battery state 
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
                 const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - frontReachingStartTime;
                 std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
                 std::cout <<" Interrupted time(due to reach ends) is: " << elapsed.count() << "seconds(s)" << std::endl;
                 frontReachingStartTime = std::chrono::high_resolution_clock::now();
             }
         }
 
         // If being interrupted, try to go to the original path again
         if ( has_possibleInterrupt || has_InterruptNeedToReDo ){
             if ( cur_pathReachingState.pathReadyToGo && cur_pathReachingState.pathOnGoing && has_possibleInterrupt ){
                 std::cout <<" Being interrupted and try to go again..." << std::endl;
                 cur_pathReachingState.pathOnGoing = false;
                 has_possibleInterrupt = false;
             }
 
             if ( cur_pathReachingState.pathReadyToGo && cur_pathReachingState.pathOnGoing && has_InterruptNeedToReDo ){
                 std::cout <<" Being interrupted and try to go again..." << std::endl;
                 cur_pathReachingState.pathOnGoing = false;
                 has_InterruptNeedToReDo = false;
             }
 
             auto start_time_t = std::chrono::system_clock::to_time_t(
                 std::chrono::time_point_cast<std::chrono::system_clock::duration>(frontReachingStartTime)
             );
             const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - frontReachingStartTime;
             std::cout <<" Original front reaching start time: " << std::ctime(&start_time_t) << std::endl;
             std::cout <<" Interrupted time(due to dodging) is: " << elapsed.count() << "seconds(s)" << std::endl;
             frontReachingStartTime = std::chrono::high_resolution_clock::now();
         }
 
         // Go to path
         if ( cur_pathReachingState.pathReadyToGo ){
             frontReachingStartTime = std::chrono::high_resolution_clock::now();
             if ( cur_pathReachingState.pathOnGoing == false ){
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
                 std::cout <<" Start go to action with front: " << front << std::endl;
                 cur_distToMove = front;
                 if ( cur_distToMove >= 1.0f )
                     time_toMove = 3;
                 else
                     time_toMove = 5;
                 if ( dist_to_reach > -1.0f ){
                     cur_distToMove = 0.7f;
                     time_toMove = 2;
                 }
                 Goto(od4, cur_distToMove * std::cos( cur_state.yaw ), cur_distToMove * std::sin( cur_state.yaw ), 0.0f, 0.0f, time_toMove);
                 cur_pathReachingState.pathOnGoing = true;
                 cur_pathReachingState.startFront = front;
                 on_GoTO_MODE = true;
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
                     else if ( std::abs( cur_pathReachingState.startFront - front ) >= 0.6f ){
                         std::cout <<" Reach target with 0.6 range of current front: " << front << std::endl;
                         Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
 
                         // Reset flags
                         on_GoTO_MODE = false;
                     }
                 }
                 else if ( std::abs( cur_pathReachingState.startFront - front ) >= cur_distToMove - safe_endreach_dist){
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
             }
         }
         
 
         /*
             Look around
             - Turn around for 360 degree to see whether target exist
             - In the mean time, record some possible front path to go to
         */
         // If Interrupted
         if ( has_InterruptNeedToReDo ){
             if ( cur_lookAroundState.turnStarted || cur_lookAroundState.clearPathCheckStarted ){
                 std::cout <<" Possible interruption to reset look around..." << std::endl;
                 cur_lookAroundState.turnStarted = false;
                 cur_lookAroundState.clearPathCheckStarted = false;
                 cur_lookAroundState.nTimer = 0;
                 has_InterruptNeedToReDo = false;
                 on_TURNING_MODE = false;
             }
 
             auto start_time_t = std::chrono::system_clock::to_time_t(
                 std::chrono::time_point_cast<std::chrono::system_clock::duration>(lookAroundStartTime)
             );
             const std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - lookAroundStartTime;
             std::cout <<" Original look up start time: " << std::ctime(&start_time_t) << std::endl;
             std::cout <<" Interrupted time(due to dodging) is: " << elapsed.count() << "seconds(s)" << std::endl;
             lookAroundStartTime = std::chrono::high_resolution_clock::now();
         }
 
         // Try to find a clear path close to the target
         // std::cout <<" Run to here with clearPathCheckStarted: " << cur_lookAroundState.clearPathCheckStarted << std::endl;
         if ( cur_lookAroundState.clearPathCheckStarted == false ){
             lookAroundStartTime = std::chrono::high_resolution_clock::now();
             std::cout <<" No target exist, ready to turn around to find" << std::endl;
             if ( cur_lookAroundState.nTimer == 0 ){
                 if ( angleFrontState_vec.size() > 0 ){
                     // Initialize the vector here
                     angleFrontState_vec.clear();
                 }
                 cur_lookAroundState.preAngle = cur_state.yaw + M_PI;
             }
             cur_lookAroundState.startAngle = cur_state.yaw;
             Goto(od4, 0.0f, 0.0f, 0.0f, 120.0f / 180.0f * M_PI, 2);
             cur_lookAroundState.clearPathCheckStarted = true;
             on_TURNING_MODE = true;
         }
         else if ( cur_lookAroundState.turnStarted == false ){
             if ( std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state.yaw ) ) < 110.0f / 180.0f * M_PI ){
                 if ( has_possibleInterrupt ){
                     std::cout <<" Some targets occur, so try to look up again..." << std::endl;
                     float angTurn = 120.0f / 180.0f * M_PI - std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state.yaw ) );
                     Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 2);
                     on_TURNING_MODE = true;
                     has_possibleInterrupt = false;
                     continue;
                 }
                 
                 // std::cout <<" Record target with angle dev: " << std::abs( angleDifference( cur_lookAroundState.startAngle, cur_state.yaw ) ) << ", and vector size: " << angleFrontState_vec.size() << std::endl;
                 // Record the angle and front
                 angleFrontState state;
                 state.front = front;
                 state.angle = wrap_angle(cur_state.yaw);
                 angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                 
                 state.front = rear;
                 state.angle = wrap_angle(cur_state.yaw + M_PI);
                 angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
 
                 state.front = left;
                 state.angle = wrap_angle(cur_state.yaw + M_PI / 2.0f);
                 angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
 
                 state.front = right;
                 state.angle = wrap_angle(cur_state.yaw - M_PI / 2.0f);
                 angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
                 // std::cout <<" Add to vector... " << std::endl;
             }
             else{
                 // Stop the turning action
                 Goto(od4, 0.0f, 0.0f, 0.0f, 0.0f, 0, 1, false);
 
                 if ( cur_lookAroundState.nTimer < 2 ){
                     cur_lookAroundState.clearPathCheckStarted = false;
                     cur_lookAroundState.nTimer += 1;
                     continue;
                 }
                 else{
                     std::this_thread::sleep_for(std::chrono::milliseconds(500));
                     cur_lookAroundState.nTimer = 0;
                 }
 
                 // Sort the angle array first 
                 if ( cur_lookAroundState.smallToBig == false ){
                     std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                         if( a.front != b.front )
                             return a.front > b.front;
                     });
                     cur_lookAroundState.smallToBig = true;
                 }
                 else{
                     std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                         if( a.front != b.front )
                             return a.front < b.front;
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
 
                         // Try to turn to the angle
                         // std::cout <<" Current angle: " << cur_state.yaw << std::endl;
                         // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                         // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                         // std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) << std::endl;
                         float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                         if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                             angTurn -= 10.0f / 180.0f * M_PI;
                         }
                         Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 1);
                         std::cout <<" Found a path to go to and start turning to target angle with target: " << cur_lookAroundState.targetAngle << std::endl;
                         cur_lookAroundState.turnStarted = true;
                         break;
                     }
                 }
 
                 // If turning still not starting, we return to previous angle
                 if ( cur_lookAroundState.turnStarted == false ){
                     // Found the path, turn to that angle
                     cur_lookAroundState.targetAngle = cur_lookAroundState.preAngle;
 
                     // Try to turn to the angle
                     // std::cout <<" Current angle: " << cur_state.yaw << std::endl;
                     // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                     // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                     // std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) << std::endl;
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
                 if ( has_possibleInterrupt ){
                     std::cout <<" Some targets occur, so try to turn to the target look up angle again..." << std::endl;
                     float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                     if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                         angTurn -= 10.0f / 180.0f * M_PI;
                     }
                     Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                     on_TURNING_MODE = true;
                     has_possibleInterrupt = false;
                     continue;
                 }
                 
                 // continue turning
                 // std::cout <<" Keep turning to that angle with current angle: " << cur_state.yaw << std::endl;
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
                     safe_dist = safe_endreach_dist + front / 3 + 0.35f;
                 else
                     safe_dist = safe_endreach_dist;
                 if ( front <= safe_dist ){
                     std::cout <<" Turn to the target angle, But current angle is not goable..." << std::endl;
                     angleFrontState state;
                     state.front = front;
                     state.angle = wrap_angle(cur_state.yaw);
                     angleFrontState_vec.insert(angleFrontState_vec.begin(),state);
 
                     // Start to find another way to go to
                     // Sort the angle array first 
                     if ( cur_lookAroundState.smallToBig == false ){
                         std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                             if( a.front != b.front )
                                 return a.front > b.front;
                         });
                         cur_lookAroundState.smallToBig = true;
                     }
                     else{
                         std::sort(angleFrontState_vec.begin(), angleFrontState_vec.end(), [](const angleFrontState& a, const angleFrontState& b) {
                             if( a.front != b.front )
                                 return a.front < b.front;
                         });
                         cur_lookAroundState.smallToBig = false;
                     }
 
                     // Check for clear path
                     std::cout <<" Start path checking..." << std::endl;
                     bool hasObOnPath = false; 
                     for ( const auto& pair_cand : angleFrontState_vec ){
                         if ( pair_cand.front <= safe_endreach_dist + pair_cand.front / 5 ){
                             continue;
                         }
 
                         if ( std::abs( angleDifference( cur_lookAroundState.preAngle, cur_state.yaw ) ) > 10.0f / 180 * M_PI ){
                             if ( std::abs( angleDifference( cur_lookAroundState.preAngle, pair_cand.angle ) ) <= 10.0f / 180 * M_PI ){
                                 continue;
                             }
                         }
 
                         if ( std::abs( angleDifference( pair_cand.angle, cur_state.yaw ) ) <= 1.0 / 180.0f * M_PI ){
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
 
                         if ( hasObOnPath == false && std::abs( angleDifference( cur_lookAroundState.preAngle, cur_state.yaw ) ) > 10.0f / 180 * M_PI ){
                             // Found the path, turn to that angle
                             cur_lookAroundState.targetAngle = pair_cand.angle;
 
                             // Try to turn to the angle
                             // std::cout <<" Current angle: " << cur_state.yaw << std::endl;
                             // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                             // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                             // std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) << std::endl;
                             float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                             if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                                 angTurn -= 10.0f / 180.0f * M_PI;
                             }
                             Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                             std::cout <<" Found a path to go to and start turning to target angle with target: " << cur_lookAroundState.targetAngle << std::endl;
                             break;
                         }
                     }
 
                     // If turning still not starting, we return to previous angle
                     if ( hasObOnPath == false ){
                         continue;
                     }
 
                     // Found the path, turn to that angle
                     cur_lookAroundState.targetAngle = cur_lookAroundState.preAngle;
 
                     // Try to turn to the angle
                     // std::cout <<" Current angle: " << cur_state.yaw << std::endl;
                     // std::cout <<" Target angle: " << cur_lookAroundState.targetAngle << std::endl;
                     // std::cout <<" Vector size: " << angleFrontState_vec.size() << std::endl;
                     // std::cout <<" Angle difference: " << angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) << std::endl;
                     float angTurn = angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) + 5.0f / 180.0f * M_PI;
                     if ( angleDifference( cur_state.yaw, cur_lookAroundState.targetAngle ) < 0.0f ){
                         angTurn -= 10.0f / 180.0f * M_PI;
                     }
                     Goto(od4, 0.0f, 0.0f, 0.0f, angTurn, 3);
                     std::cout <<" No path to go to so start turning to the previous target angle with target: " << cur_lookAroundState.targetAngle << std::endl;
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
             }
         }   
     }
 
     retCode = 0;
     return retCode;
 }