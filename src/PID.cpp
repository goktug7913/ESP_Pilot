#include "PID.h"

extern FC_cfg cfg; // global configuration

PID::PID()
{
	// Default constructor
}

PID::PID(float proportionalGain, float integralGain, float derivativeGain)
{
	// Constructor with initial PID gains
	this->proportionalGain = proportionalGain;
	this->integralGain = integralGain;
	this->derivativeGain = derivativeGain;
}

float PID::Calculate(float processVariable, float target, float deltaTime)
{
	this->deltaTime = deltaTime;

	float error = target - processVariable;

	if (abs(error) > ITERMDEADBAND)
	{ // some deadband
		this->integral += (this->integralGain * error * this->integralScalar * this->deltaTime);
	}

	float derivative = this->derivativeGain * ((error - this->previousError) / this->deltaTime);

	this->integral = clamp(this->integral, this->iMin, this->iMax);
	this->previousError = error;

	return (this->proportionalGain * error) + this->integral + derivative;
}

void PID::SetGains(float proportionalGain, float integralGain, float derivativeGain)
{
	this->proportionalGain = proportionalGain;
	this->integralGain = integralGain;
	this->derivativeGain = derivativeGain;
}

float PID::clamp(float value, float min, float max)
{
	return std::max(min, std::min(max, value));
}