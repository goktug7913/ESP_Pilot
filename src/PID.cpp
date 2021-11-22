#include "PID.h"

extern FC_cfg cfg; //

PID::PID(){
	// Default constructor
}

PID::PID(float p_gain, float i_gain, float d_gain) {
	// Constructor with initial PID gains
	Kp = p_gain;
	Ki = i_gain;
	Kd = d_gain;
}

float PID::Calculate(float pv, float target, float dt) {
	dT = dt;
	p = target - pv;
	i += p * dT;
	d = (p - p_prev) / dT;
	
	p_prev = p;
  
  if (i>iMax){i = iMax;}
  if (i<iMin){i = iMin;}

  //TODO: add some deadzone to integral term to prevent oscillations and windup

	return (Kp * p) + (Ki * i) + (Kd * d);
} 