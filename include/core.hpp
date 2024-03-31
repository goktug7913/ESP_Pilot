#include "control/pid.hpp"

class EsppCore {
private:
    bool bInputEnable = 1;
    bool bOutputEnable = 0;
    bool bStepEnable = 0;
    bool bSensorsEnable = 0;

    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidYaw;

public:
    EsppCore();
    void init();
    void start();
};