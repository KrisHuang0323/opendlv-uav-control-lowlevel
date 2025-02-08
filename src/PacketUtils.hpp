#pragma once
#include "crazyflieLinkCpp/Connection.h"

class PacketUtils
{
public:
    // Constructs a high level commander goto packet
    // const uint8_t gotoSize = 25;
    static std::array<uint8_t, 25> gotoCommand(float x, float y, float z, float yaw, float time) {
        std::array<uint8_t, 25> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)12);    // Command (Goto = 12)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, true);          // Set to true, if position/yaw are relative to current setpoint
        index += pack(buffer.data(), index, true);          // Set to true for linear interpolation instead of smooth polynomial
        index += pack(buffer.data(), index, x);             // x
        index += pack(buffer.data(), index, y);             // y
        index += pack(buffer.data(), index, z);             // z
        index += pack(buffer.data(), index, yaw);           // Yaw
        index += pack(buffer.data(), index, time);          // Duration (seconds)
        return buffer;
    }

    // Constructs a high level commander takeoff packet
    // const uint8_t takeoffSize = 16;
    static std::array<uint8_t, 16> takeoffCommand(float height, float yaw, float time) {
        std::array<uint8_t, 16> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)7);    // Command (Takeoff = 7)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, height);        // Height
        index += pack(buffer.data(), index, yaw);           // Yaw
        index += pack(buffer.data(), index, true);          // Use Current Yaw
        index += pack(buffer.data(), index, time);          // Duration (seconds)
        return buffer;
    }

    // Constructs a high level commander land packet
    // const uint8_t landSize = 16;
    static std::array<uint8_t, 16> landCommand(float height, float yaw, float time) {
        std::array<uint8_t, 16> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)8);    // Command (Land = 8)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, height);        // Height
        index += pack(buffer.data(), index, yaw);           // Yaw
        index += pack(buffer.data(), index, true);          // Use Current Yaw
        index += pack(buffer.data(), index, time);          // Duration (seconds)
        return buffer;
    }

    // Constructs a high level commander stop packet
    // const uint8_t stopSize = 3;
    static std::array<uint8_t, 3> stopCommand() {        
        std::array<uint8_t, 3> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)3);    // Command (Stop = 3)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        return buffer;
    }

private:
    static size_t pack(uint8_t* buffer, uint8_t index, float value) {
        union {
            float value;
            unsigned char bytes[4];
        } converter;
        converter.value = value;
        buffer[index + 0] = converter.bytes[0];
        buffer[index + 1] = converter.bytes[1];
        buffer[index + 2] = converter.bytes[2];
        buffer[index + 3] = converter.bytes[3];
        return 4;
    }

    static size_t pack(uint8_t* buffer, uint8_t index, uint8_t value) {
        buffer[index] = value;
        return 1;
    }

    static size_t pack(uint8_t* buffer, uint8_t index, bool value) {
        uint8_t byteVal = (value) ? 1 : 0;
        buffer[index] = byteVal;
        return 1;
    }
};