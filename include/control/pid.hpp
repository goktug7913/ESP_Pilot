#ifndef PID_HPP
#define PID_HPP

struct PIDCoefficients {
    float kp;
    float ki;
    float kd;
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    PIDController(PIDCoefficients coefficients);
    float calculate(float setpoint, float measured_value);
    void setCoefficients(PIDCoefficients coefficients);
    void reset();

private:
    PIDCoefficients coefficients_;
    float integral_;
    float previous_error_;
};

#endif // PID_HPP
