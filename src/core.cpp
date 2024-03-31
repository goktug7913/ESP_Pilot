#include "core.hpp"

EsppCore::EsppCore() : pidRoll(0.0f, 0.0f, 0.0f), pidPitch(0.0f, 0.0f, 0.0f), pidYaw(0.0f, 0.0f, 0.0f) {}

void EsppCore::init() {
    // Initialize system components
    // This would include setting up RTOS tasks, initializing drivers, etc.
    pidRoll.setCoefficients(1.0f, 0.0f, 0.0f);
    pidPitch.setCoefficients(1.0f, 0.0f, 0.0f);
    pidYaw.setCoefficients(1.0f, 0.0f, 0.0f);
}

void EsppCore::start() {
    // Start system components
    // This would include starting RTOS tasks, enabling drivers, etc.
    bInputEnable = 1;
    bOutputEnable = 1;
    bStepEnable = 1;
    bSensorsEnable = 1;
}