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
 
template <size_t N>
std::string arrayToString(const std::array<uint8_t, N>& arr) {
    std::ostringstream oss;
    for (size_t i = 0; i < N; ++i) {
        oss << static_cast<uint8_t>(arr[i]) << " ";  // 轉為 int 避免 char 解析問題
    }
    return oss.str(); // 會有尾部空格
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << "You should include the cid to start communicate in OD4Session" << std::endl;
        return retCode;
    }

    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    // // Handler to receive distance readings (realized as C++ lambda).
    // std::mutex distancesMutex;
    // float front{0};
    // float rear{0};
    // float left{0};
    // float right{0};
    // auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
    //     auto senderStamp = env.senderStamp();
    //     // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
    //     opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
    //     // Store distance readings.
    //     std::lock_guard<std::mutex> lck(distancesMutex);
    //     switch (senderStamp) {
    //         case 0: front = dr.distance(); break;
    //         case 2: rear = dr.distance(); break;
    //         case 1: left = dr.distance(); break;
    //         case 3: right = dr.distance(); break;
    //     }
    // };
    // // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    // od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

    // Endless loop; end the program by pressing Ctrl-C.
    if (od4.isRunning()) {
        ////////////////////////////////////////////////////////////////
        // Example for creating and sending a message to other microservices; can
        // be removed when not needed.
        // Requires high level commander enabled (param commander.enHighLevel == 1)
        std::cout << "Taking off..." << std::endl;        
        cluon::data::TimeStamp sampleTime;
        opendlv::logic::action::CrazyFlieCommand cfcommand;
        cfcommand.height(0.5f);
        cfcommand.yaw(0.0f);
        cfcommand.time(3.0f);
        // auto takeoffArray = PacketUtils::takeoffCommand(0.5f, 0.0f, 3.0f);   
        // std::string strCommand = arrayToString(takeoffArray);
        // // std::string strCommand;
        // // strCommand.assign(takeoffArray.begin(), takeoffArray.end());          
        // uint32_t takeoffSize = takeoffArray.size();  
        // std::cout << "Command to send: " << strCommand << ", size: " << takeoffSize << std::endl;   
        // cfcommand.size(takeoffSize);
        // cfcommand.command(strCommand);
        od4.send(cfcommand, sampleTime, 0);
        // cf.getCon().send(PacketUtils::takeoffCommand(0.5f, 0.0f, 3.0f));
        std::this_thread::sleep_for(std::chrono::milliseconds(3250));

        std::cout << "Go to..." << std::endl;
        // auto gotoArray = PacketUtils::gotoCommand(0.5f, 0.5f, 1.0f, 0.0f, 3.0f);
        // strCommand = arrayToString(gotoArray);
        // uint32_t gotoSize = gotoArray.size();
        cfcommand.x(0.5f);
        cfcommand.y(0.5f);
        cfcommand.z(1.0f);
        cfcommand.yaw(0.0f);
        cfcommand.time(3.0f);
        od4.send(cfcommand, sampleTime, 3);
        // cf.getCon().send(PacketUtils::gotoCommand(0.5f, 0.5f, 1.0f, 0.0f, 3.0f));
        std::this_thread::sleep_for(std::chrono::milliseconds(3250));

        std::cout << "Landing..." << std::endl;
        // auto landArray = PacketUtils::landCommand(0.0f, 0.0f, 3.0f);
        // strCommand = arrayToString(landArray);
        cfcommand.height(0.0f);
        cfcommand.yaw(0.0f);
        cfcommand.time(3.0f);
        od4.send(cfcommand, sampleTime, 0);
        // cf.getCon().send(PacketUtils::landCommand(0.0f, 0.0f, 3.0f));
        std::this_thread::sleep_for(std::chrono::milliseconds(3250));

        std::cout << "Stopping..." << std::endl;
        // auto stopArray = PacketUtils::stopCommand();
        od4.send(cfcommand, sampleTime, 2);
        // cf.getCon().send(PacketUtils::stopCommand());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    retCode = 0;
    return retCode;
}

