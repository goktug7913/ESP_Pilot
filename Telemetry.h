#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <array>
#include "Config.h"
#pragma once



struct telemetry_frame {
  int header = 151522;

  std::array<float, 3> gyro;
  std::array<float, 3> accel;
  std::array<float, 3> rx_scaled;
  std::array<uint16_t, pwm_ch_amt> rx_raw;

  float pid_p, pid_r, pid_y;
  int esc1_out, esc2_out, esc3_out, esc4_out;

};

class TelemetryManager{

  public:

  void StartRecording();
  void StopRecording();

  void StartSerial();
  void StopSerial();
  
  void SerialSendFrame();

  bool enableserial = 0;
  bool enablerecording = 0;

  private:

  void SaveFrame();

  telemetry_frame current_frame;

};

#endif