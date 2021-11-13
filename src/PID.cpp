#include "PID.h"

extern FC_cfg cfg;

PID::PID(){}

PID::PID(float p_gain, float i_gain, float d_gain) {
	Kp = p_gain;
	Ki = i_gain;
	Kd = d_gain;
}

float PID::Calculate(float pv, float target) {
	p = target - pv;
	i += p * dT;
	d = (p - p_prev) / dT;
	
	p_prev = p;
  
  if (i>iMax){i = iMax;}
  if (i<iMin){i = iMin;}

	return p * Kp + i * Ki + d * Kd;
} 