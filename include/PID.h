#include "Config.h"
#include "definitions.h"
#include <array>
#pragma once

class PID {
  public:
  
  PID();
  PID(float p_gain, float i_gain, float d_gain);

  // Public Functions of the PID instance

  float               Calculate(float pv, float target, float dt);
  void                SetGains(float p_gain, float i_gain, float d_gain);
  std::array<float,3> GetGains();


  //TO-DO: move to private
  float Kp, Ki, Kd;   //PID Gains
  float p, i, d;      //Error values
  float p_prev;       //Previous P Term
  float dT = 0;       //Delta time in millisecs

  float iMax =  35000;
  float iMin = -35000;
};