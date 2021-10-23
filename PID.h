#include "Controller.h"
#include "Config.h"
#include "definitions.h"
#include "Arduino.h"
#pragma once

class PID {
  public:

  int     throttle;                                 // Throttle signal
  int     pid_esc1, pid_esc2, pid_esc3, pid_esc4;   // Final PWM Signals combined with throttle for each ESC

  float   pid_pitch, pid_roll, pid_yaw;
  float   rx_angles[3];                             //roll pitch yaw

  float   pErr_pitch, pErr_roll, pErr_yaw;          // P-Term Errors
  float   iErr_pitch, iErr_roll, iErr_yaw;          // I-Term Errors
  float   dErr_pitch, dErr_roll, dErr_yaw;          // D-Term Errors

  float   iErr_pitch_prev, iErr_roll_prev, iErr_yaw_prev; // Previous I-Term Errors

  unsigned long Tstart, Tend;

  void calculatePid();
  void calculateTargets();
};