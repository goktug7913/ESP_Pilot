#include "PID.h"

extern FC FliCon;
extern FC_cfg cfg;

void PID::calculatePid(){
  

  pid_esc1 = pid_esc2 = pid_esc3 = pid_esc4 = 0;
  throttle = FliCon.rx_raw[2];

  // Calculate Roll PWM
  if (pErr_roll < 0){
    pid_esc1 += pErr_roll;  pid_esc2 -= pErr_roll;
    pid_esc3 += pErr_roll;  pid_esc4 -= pErr_roll;
  } else {
    pid_esc2 -= pErr_roll;  pid_esc1 += pErr_roll;
    pid_esc4 -= pErr_roll;  pid_esc3 += pErr_roll;
  }

  // Calculate Pitch PWM
  if (pErr_pitch > 0){
    pid_esc1 -= pErr_pitch;  pid_esc3 += pErr_pitch;
    pid_esc2 -= pErr_pitch;  pid_esc4 += pErr_pitch;
  } else {
    pid_esc3 += pErr_pitch;  pid_esc1 -= pErr_pitch;
    pid_esc4 += pErr_pitch;  pid_esc2 -= pErr_pitch;
  }
  // Add Throttles
  pid_esc1 += throttle; pid_esc2 += throttle; pid_esc3 += throttle; pid_esc4 += throttle;

  // Limit PWM signals so ESC's won't complain
  if (pid_esc1 < 1000) {pid_esc1 = 1000;} else if(pid_esc1 > 2000) {pid_esc1 = 2000;}
  if (pid_esc2 < 1000) {pid_esc2 = 1000;} else if(pid_esc2 > 2000) {pid_esc2 = 2000;}
  if (pid_esc3 < 1000) {pid_esc3 = 1000;} else if(pid_esc3 > 2000) {pid_esc3 = 2000;}
  if (pid_esc4 < 1000) {pid_esc4 = 1000;} else if(pid_esc4 > 2000) {pid_esc4 = 2000;}
}

void PID::calculateTargets(){
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP ROLL SIGNAL TO DEGREE TARGET

  if (FliCon.rx_raw[3] >= PWM_CENTER+cfg.rx_deadzone){
    rx_angles[0] = map( FliCon.rx_raw[3],  PWM_CENTER,  2000,  0,  cfg.max_angle );
    pErr_roll = (FliCon.gyro[1] + rx_angles[0]) * cfg.p_gain;
  }
    
  else if (FliCon.rx_raw[3] < PWM_CENTER-cfg.rx_deadzone){
    rx_angles[0] = map( FliCon.rx_raw[3],  PWM_CENTER,  1000,  0,  cfg.max_angle );
    pErr_roll = 0-(FliCon.gyro[1] + rx_angles[0]) * cfg.p_gain;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP PITCH SIGNAL TO DEGREE TARGET

  if (FliCon.rx_raw[1] >= PWM_CENTER+cfg.rx_deadzone){
    rx_angles[1] = map( FliCon.rx_raw[1],  PWM_CENTER,  2000,  0,  cfg.max_angle );
    pErr_pitch = (FliCon.gyro[0] + rx_angles[1]) * cfg.p_gain;
  }
    
  else if (FliCon.rx_raw[1] < PWM_CENTER-cfg.rx_deadzone){
    rx_angles[1] = map( FliCon.rx_raw[1],  PWM_CENTER,  1000,  0,  cfg.max_angle );
    pErr_pitch = 0-(FliCon.gyro[0] + rx_angles[1]) * cfg.p_gain;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP YAW SIGNAL TO DEGREE TARGET

  if (FliCon.rx_raw[0] >= PWM_CENTER+cfg.rx_deadzone){
    rx_angles[2] = map( FliCon.rx_raw[0],  PWM_CENTER,  2000,  0,  cfg.max_angle );
    pErr_yaw = (FliCon.gyro[2] + rx_angles[2]) * cfg.p_gain;
  }
    
  else if (FliCon.rx_raw[0] < PWM_CENTER-cfg.rx_deadzone){
    rx_angles[2] = map( FliCon.rx_raw[0],  PWM_CENTER,  1000,  0,  cfg.max_angle );
    pErr_yaw = (FliCon.gyro[2] + rx_angles[2]) * cfg.p_gain;
  }
  calculatePid();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //DEBUG 
  /*Serial.printf("Roll Err: %.2f - ",  pErr_roll);
  Serial.printf("Pitch Err: %.2f - ", pErr_pitch);
  Serial.printf("Yaw Err: %.2f - ",   pErr_yaw);
  Serial.print('\n');
  Serial.printf("Esc1:%d, Esc2:%d, Esc3:%d, Esc4:%d,", pid_esc1, pid_esc2, pid_esc3, pid_esc4);*/
  }