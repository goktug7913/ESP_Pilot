#pragma once
#include "control/pid.hpp"
#include "control/input.hpp"

class EsppCore {
private:
    bool bInputEnable = true;
    bool bOutputEnable = false;
    bool bStepEnable = false;
    bool bSensorsEnable = true;

    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidYaw;
    InputSystem input;

public:
    EsppCore();
    void init();
    void start();
};