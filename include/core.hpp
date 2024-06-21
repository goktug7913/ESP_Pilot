#pragma once
#include "config/config_manager.hpp"
#include "control/input.hpp"
#include "control/pid.hpp"
#include "control/mixer.hpp"

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
    Mixer mixer;
    
    void init();
    
public:
    EsppCore();
    void start();
};