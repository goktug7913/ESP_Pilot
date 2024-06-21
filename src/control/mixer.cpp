#include "control/mixer.hpp"

Mixer::Mixer(MotorGeometry geometry) {
    motorGeometry = geometry;
}

void Mixer::mix(float roll, float pitch, float yaw, float throttle, float* motor_out) {
    // This is a simple mixer for a quadcopter
    // The mixer is based on the motor geometry
    // The motor geometry is defined in the config file
    // The mixer is a simple linear mixer
}