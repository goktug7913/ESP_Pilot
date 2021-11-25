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


	if(p > ITERMDEADBAND || p < ITERMDEADBAND){ //some deadband
		i += (p * iScalar * dT);
	}

	d = (p - p_prev) / dT;
  
  	if (i>iMax){i = iMax;}
  	if (i<iMin){i = iMin;}
	p_prev = p;

	return (Kp * p) + (Ki * i) + (Kd * d);
} 