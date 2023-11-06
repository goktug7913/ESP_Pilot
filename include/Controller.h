#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <array>
#include "Config.h"
#include "PID.h"
#include <MPU6050_light.h>
#include "driver/mcpwm.h"
#include "ConfigSuite.h"
#pragma once



class FC{
  public:

  std::array<float, 3> gyro{};
  std::array<float, 3> accel{};
  std::array<float, 3> rx_scaled{};
  std::array<uint16_t, pwm_ch_amt> rx_raw{};

  float pid_p{}, pid_r{}, pid_y{};
  int esc1_out{}, esc2_out{}, esc3_out{}, esc4_out{};

  float temperature{};
  bool armed = false;
  bool recovery = false;
  bool usbmode = false;
  bool alt_hold = false;
  bool gps_hold = false;
  float dt{};
  float t1{};
  
  PID pitch_pid;
  PID roll_pid;
  PID yaw_pid;
  //PID alt_pid;

  FC();

  void Start();
  void InputTransform();
  void OutputTransform();
  void MotionUpdate();
  void writeEsc(uint32_t esc1, uint32_t esc2, uint32_t esc3, uint32_t esc4);

 
  void disarm();
  void setGains(float p, float i, float d);
};

#endif