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


void Takeoff(cluon::OD4Session &od4, float height, float duration){
    std::cout << "Taking off to height: " << height << std::endl;        
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 0);
}

void Goto(cluon::OD4Session &od4, float x, float y, float z, float yaw, float duration){
    std::cout << "Go to position : x: "<< x << " ,y: " << y << " ,z: " << z << " , yaw: " << yaw << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.x(x);
    cfcommand.y(y);
    cfcommand.z(z);
    cfcommand.yaw(yaw);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 3);
}

void Landing(cluon::OD4Session &od4, float height, float duration){
    std::cout << "Landing to height: " << height << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    cfcommand.height(height);
    cfcommand.time(duration);
    od4.send(cfcommand, sampleTime, 1);
}

void Stopping(cluon::OD4Session &od4){
    std::cout << "Stopping." << std::endl;
    cluon::data::TimeStamp sampleTime;
    opendlv::logic::action::CrazyFlieCommand cfcommand;
    od4.send(cfcommand, sampleTime, 2);
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
    while (od4.isRunning()) {
        // Sleep for 100 ms to not let the loop run to fast
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Takeoff first
        if ( hasTakeoff == false ){
            Takeoff(od4, 0.5f, 1.0f);
            hasTakeoff = true;
        }

        /*
            Obstacle avoidance
        */
        // Check that whether the crazyflie is too close to something
        // If left and right less than 0.5 m
        if ( left <= safe_dist || right <= safe_dist ){
            // Move away from wall until left and right larger than 0.5
            float dist = 0.1f;
            if ( left <= safe_dist ){
                // Move right
                Goto(od4, 0.0f, -dist, 0.0f, 0.0f, 3.0f);
            }
            else if ( right <= safe_dist ){
                // Move left
                Goto(od4, 0.0f, dist, 0.0f, 0.0f, 3.0f);
            }
            continue;
        }

        // Check that whether there are purple balls occur in the vision
        /*
            Homing
        */

        /*
            Target reaching
        */
        // Reset front looking dist while some target occur

        
        /*
            Finding
        */
       // If the front is larger than the safe dist and the turning mode is off, move to the destination
       if ( front > safe_dist && Turning_mode == false ){
            float dist_to_move = 0.1f;
            Goto(od4, dist_to_move, 0.0f, 0.0f, 0.0f, 1.0f);
            dist_moved += dist_to_move;
            Timer += 1;
            continue;
        }           

        // Turn the turning mode on
        if ( Turning_mode == false ){
            Turning_mode = true;
        }
        
        // Record dist moved
        if ( dist_moved != 0.0f && dist_moved_so_far < dist_moved ){
            dist_moved_so_far = dist_moved;
            dist_moved = 0;
        }     

        // Change the front looking distance while the dist moved got stucked
        if ( Timer >= timer_max && dist_moved_so_far <= front_looking_dist ){
            front_looking_dist += 0.5f;
            dist_moved_so_far = 0;
            Timer = 0;
        }

        // If the crazyflie has already turned around and can not find anything, shorten the front looking dist
        if ( angle_moved >= 2 * M_PI ){
            front_looking_dist -= 0.2f;
            angle_moved = 0;
        }
        
        // Tuning around to find somewhere to go to 
        if ( front <= front_looking_dist ){
            // Turn around to see whether somewhere can go
            float angle = 5.0f / 180.0f * M_PI;
            Goto(od4, 0.0f, 0.0f, 0.0f, angle, 1.0f);
            angle_moved += angle;
        }
        else{
            // Turn off turning mode while some directions found
            Turning_mode = false;

            // Reset moved angle
            angle_moved = 0;
        }
    }

    retCode = 0;
    return retCode;
}