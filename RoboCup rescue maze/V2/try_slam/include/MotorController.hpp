#pragma once
#include "SerialBus.hpp"
#include "RobotConfig.hpp"

class MotorController {
public:
    MotorController(SerialBus& bus, const RobotConfig& config);
    void setSpeed(float left, float right); // -maxSpeed .. maxSpeed

private:
    SerialBus& bus;
    const RobotConfig& config;
};
