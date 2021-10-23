#include <array>
#include "Config.h"
#pragma once


typedef struct{
  std::array<float, 3> gyro;
  std::array<float, 3> gyro_rps;
  std::array<float, 3> accel;
  std::array<uint16_t, pwm_ch_amt> rx_raw;
  std::array<uint16_t, pwm_ch_amt> rx_scaled;

  float temperature;
  bool armed = 0;
  bool recovery = 0;
  bool usbmode = 0;
} FC;