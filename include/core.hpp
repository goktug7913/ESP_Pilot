#include "control/pid.hpp"

class EsppCore {
private:
    bool bInputEnable = true;
    bool bOutputEnable = false;
    bool bStepEnable = false;
    bool bSensorsEnable = true;

    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidYaw;

public:
    EsppCore();
    void init();
    void start();
};