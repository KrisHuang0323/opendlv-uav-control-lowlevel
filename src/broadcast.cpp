#include <iostream>
#include <thread>

#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"
#include "Crazyflie.h"

using namespace bitcraze::crazyflieLinkCpp;

int main()
{
    /* Creates a connection that will broadcast to all
     * crazyflies on channel 80. Note that this requires
     * updating the Crazyradio firmware from its factory version
     */
    
    Crazyflie cf("radio://0/90/2M/E7E7E7E7E7");
    cf.init();

    if (!cf.isRunning())
    {
        std::cerr << "Failed to initialize Crazyflie" << std::endl;
        return -1;
    }
    // Connection broadcastConnection("radiobroadcast://*/90/2M");

    // Requires high level commander enabled (param commander.enHighLevel == 1)
    std::cout << "Taking off..." << std::endl;
    cf.getCon().send(PacketUtils::takeoffCommand(0.5f, 0.0f, 3.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(3250));

    std::cout << "Go to..." << std::endl;
    cf.getCon().send(PacketUtils::gotoCommand(0.5f, 0.5f, 1.0f, 0.0f, 3.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(3250));

    std::cout << "Landing..." << std::endl;
    cf.getCon().send(PacketUtils::landCommand(0.0f, 0.0f, 3.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(3250));

    std::cout << "Stopping..." << std::endl;
    cf.getCon().send(PacketUtils::stopCommand());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    cf.getCon().close();
    std::cout << "Done." << std::endl;
    return 0;
}