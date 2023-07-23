#include "Telemetry.h"
#include "Controller.h"

extern FC FliCon;

void TelemetryManager::StartRecording(){
  enablerecording = 1;
}

void TelemetryManager::StopRecording(){
  enablerecording = false;
}

void TelemetryManager::StartSerial(){
  enableserial = 1;
}

void TelemetryManager::StopSerial(){
  enableserial = 0;
}

void TelemetryManager::SerialSendFrame(){
  SaveFrame();
  // SerialMan.SendMsg(TMTY_DATA_FLAG,0,(uint8_t*)&current_frame);
}

void TelemetryManager::SaveFrame(){

  current_frame.gyro = FliCon.gyro;
  current_frame.accel = FliCon.accel;
  current_frame.rx_scaled = FliCon.rx_scaled;
  current_frame.rx_raw = FliCon.rx_raw;

  current_frame.deltaT = FliCon.dt;

  current_frame.pid_p = FliCon.pid_p;
  current_frame.pid_r = FliCon.pid_r;
  current_frame.pid_y = FliCon.pid_y;

  current_frame.esc1_out = FliCon.esc1_out;
  current_frame.esc2_out = FliCon.esc2_out;
  current_frame.esc3_out = FliCon.esc3_out;
  current_frame.esc4_out = FliCon.esc4_out;

  current_frame.p_p = FliCon.pitch_pid.p;
  current_frame.p_i = FliCon.pitch_pid.i;
  current_frame.p_d = FliCon.pitch_pid.d;

  current_frame.r_p = FliCon.roll_pid.p;
  current_frame.r_i = FliCon.roll_pid.i;
  current_frame.r_d = FliCon.roll_pid.d;

  current_frame.y_p = FliCon.yaw_pid.p;
  current_frame.y_i = FliCon.yaw_pid.i;
  current_frame.y_d = FliCon.yaw_pid.d;
}