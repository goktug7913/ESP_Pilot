#pragma once
#include "control/pid.hpp"
#include "control/input.hpp"
#include "config/config_manager.hpp"

class EsppCore {
private:
    bool bInputEnable = true;
    bool bOutputEnable = false;
    bool bStepEnable = false;
    bool bSensorsEnable = true;

    ConfigManager config;
    InputSystem input;
    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidYaw;
    

    void init();
    
public:
    EsppCore();
    void start();
};