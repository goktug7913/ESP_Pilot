#include "Telemetry.h"
#include "Controller.h"
#include "SerialManager.h"

extern FC FliCon;
extern SerialMgr SerialMan;

void TelemetryManager::StartRecording(){
  enablerecording = 1;
}

void TelemetryManager::StopRecording(){
  enablerecording = 0;
}

void TelemetryManager::StartSerial(){
  enableserial = 1;
}

void TelemetryManager::StopSerial(){
  enableserial = 0;
}

void TelemetryManager::SerialSendFrame(){
  SaveFrame();
  SerialMan.SendMsg(TMTY_DATA_FLAG,0,(uint8_t*)&current_frame);
  //Serial.write((byte*)&current_frame, sizeof(telemetry_frame));
}

void TelemetryManager::SaveFrame(){

  current_frame.gyro = FliCon.gyro;
  current_frame.accel = FliCon.accel;
  current_frame.rx_scaled = FliCon.rx_scaled;
  current_frame.rx_raw = FliCon.rx_raw;

  current_frame.pid_p = FliCon.pid_p;
  current_frame.pid_r = FliCon.pid_r;
  current_frame.pid_y = FliCon.pid_y;

  current_frame.esc1_out = FliCon.esc1_out;
  current_frame.esc2_out = FliCon.esc2_out;
  current_frame.esc3_out = FliCon.esc3_out;
  current_frame.esc4_out = FliCon.esc4_out;

}