#ifndef PID_HPP
#define PID_HPP

struct PIDCoefficients {
    float kp;
    float ki;
    float kd;
};

class PIDController {
public:
    PIDController(PIDCoefficients coefficients);
    PIDController(float kp, float ki, float kd);
    float calculate(float setpoint, float measured_value);
    void setCoefficients(PIDCoefficients coefficients);
    void setCoefficients(float kp, float ki, float kd);
    PIDCoefficients getCoefficients();
    void reset();

private:
    PIDCoefficients coefficients_;
    float integral_;
    float previous_error_;
};

#endif // PID_HPP
