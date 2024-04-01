#include "control/pid.hpp"

float PIDController::calculate(float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    integral_ += error;
    float derivative = error - previous_error_;
    previous_error_ = error;
    return coefficients_.kp * error + coefficients_.ki * integral_ + coefficients_.kd * derivative;
}

void PIDController::setCoefficients(PIDCoefficients coefficients) {
    coefficients_ = coefficients;
}

void PIDController::setCoefficients(float kp, float ki, float kd) {
    setCoefficients({kp, ki, kd});
}

PIDCoefficients PIDController::getCoefficients() {
    return coefficients_;
}

void PIDController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}

PIDController::PIDController(PIDCoefficients coefficients) {
    setCoefficients(coefficients);
}

PIDController::PIDController(float kp, float ki, float kd) {
    setCoefficients({kp, ki, kd});
}