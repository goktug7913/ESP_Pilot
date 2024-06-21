#pragma once
#include "config/types.hpp"

class Mixer {
public:
    Mixer();
    Mixer(MotorGeometry geometry);
    void mix(float roll, float pitch, float yaw, float throttle, float* motor_out);
private:
    MotorGeometry motorGeometry;
};