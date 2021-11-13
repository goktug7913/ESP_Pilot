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

  std::array<float, 3> gyro;
  std::array<float, 3> accel;
  std::array<float, 3> rx_scaled;
  std::array<uint16_t, pwm_ch_amt> rx_raw;

  float pid_p, pid_r, pid_y;
  int esc1_out, esc2_out, esc3_out, esc4_out;

  float temperature;
  bool armed = 0;
  bool recovery = 0;
  bool usbmode = 0;

  PID pitch_pid;
  PID roll_pid;
  PID yaw_pid;

  FC();

  void Start();
  void InputTransform();
  void OutputTransform();
  void MotionUpdate();
  void writeEsc(uint32_t esc1, uint32_t esc2, uint32_t esc3, uint32_t esc4);

  void parseCommand();
  void disarm();
};

#endif