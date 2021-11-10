#include "Controller.h"
#include "Telemetry.h"
#include "SerialManager.h"

extern FC_cfg cfg;
extern ConfigSuite CfgMan;
extern TelemetryManager Logger;
extern MPU6050 mpu;
extern SerialMgr SerialMan;

FC::FC(){

  pitch_pid = PID(cfg.Kp_pitch, cfg.Ki_pitch, cfg.Kd_pitch);
  roll_pid = PID(cfg.Kp_roll, cfg.Ki_roll, cfg.Kd_roll);
  yaw_pid = PID(cfg.Kp_yaw, cfg.Ki_yaw, cfg.Kd_yaw);
}

void FC::Start(){
  while(armed){
    if(rx_raw[4] < 1600 && rx_raw[4] > 1400){writeEsc(rx_raw[2], rx_raw[2], rx_raw[2], rx_raw[2]);} // Bypass PID when SWC is pos2, for ESC signal calibration
    
    SerialMan.ReceiveMsg();
    MotionUpdate();
    InputTransform();

    pid_p = pitch_pid.Calculate( gyro[0], rx_scaled[1] );
    pid_r =  roll_pid.Calculate( gyro[1], rx_scaled[0] );
    pid_y =   yaw_pid.Calculate( gyro[2], rx_scaled[2] );

    if (pid_p > 1400){pid_p = 1400;} else if (pid_p < -1400){pid_p = -1400;}
    if (pid_r > 1400){pid_r = 1400;} else if (pid_r < -1400){pid_r = -1400;}

    OutputTransform();
    
    if (rx_raw[4] < 1250){armed = 0;}  // Disarm if CH5 low
    if (Logger.enableserial){Logger.SerialSendFrame();} // Send telemetry over serial when activated
  }
}

void FC::InputTransform(){
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP ROLL SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (rx_raw[3] >= PWM_CENTER+cfg.rx_deadzone){ // Stick Right
    rx_scaled[0] = map( rx_raw[3],  PWM_CENTER,  2000,  0,  cfg.max_angle );
  }
  else if (rx_raw[3] < PWM_CENTER-cfg.rx_deadzone){ // Stick Left
    rx_scaled[0] = 0-map( rx_raw[3],  PWM_CENTER,  1000,  0,  cfg.max_angle );
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP PITCH SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (rx_raw[1] >= PWM_CENTER+cfg.rx_deadzone){ // Stick Up
    rx_scaled[1] = map( rx_raw[1],  PWM_CENTER,  2000,  0,  cfg.max_angle );
  }
  else if (rx_raw[1] < PWM_CENTER-cfg.rx_deadzone){ // Stick Down
    rx_scaled[1] = 0-map( rx_raw[1],  PWM_CENTER,  1000,  0,  cfg.max_angle );
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP YAW SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //Yaw target angle is increased or decreased by moving the stick, which
  //does not return to zero, it is kept like the last value instead.

  //The MPU library measures TOTAL yaw rotation angle in both directions,
  //so the angle target can be over 360 or below 0 degrees.

  if (rx_raw[0] >= PWM_CENTER+cfg.rx_deadzone){ // Stick Right
    rx_scaled[2] += map( rx_raw[0],  PWM_CENTER,  2000,  0,  cfg.max_angle );
  }
  else if (rx_raw[0] < PWM_CENTER-cfg.rx_deadzone){ // Stick Left
    rx_scaled[2] -= map( rx_raw[0],  PWM_CENTER,  1000,  0,  cfg.max_angle );
  }
}

void FC::OutputTransform(){
  // Calculate Roll PWM
  if (pid_r > 0){
    esc1_out += pid_r;  esc2_out -= pid_r;
    esc3_out += pid_r;  esc4_out -= pid_r;
  } else {
    esc2_out -= pid_r;  esc1_out += pid_r;
    esc4_out -= pid_r;  esc3_out += pid_r;
  }

  // Calculate Pitch PWM
  if (pid_p < 0){
    esc1_out -= pid_p;  esc3_out += pid_p;
    esc2_out -= pid_p;  esc4_out += pid_p;
  } else {
    esc3_out += pid_p;  esc1_out -= pid_p;
    esc4_out += pid_p;  esc2_out -= pid_p;
  }

  // Add Throttles
  esc1_out += rx_raw[2]; esc2_out += rx_raw[2]; esc3_out += rx_raw[2]; esc4_out += rx_raw[2];

  // Limit PWM signals so ESC's won't complain
  if (esc1_out < 1000) {esc1_out = 1000;} else if(esc1_out > 2000) {esc1_out = 2000;}
  if (esc2_out < 1000) {esc2_out = 1000;} else if(esc2_out > 2000) {esc2_out = 2000;}
  if (esc3_out < 1000) {esc3_out = 1000;} else if(esc3_out > 2000) {esc3_out = 2000;}
  if (esc4_out < 1000) {esc4_out = 1000;} else if(esc4_out > 2000) {esc4_out = 2000;}

  writeEsc(esc1_out, esc2_out, esc3_out, esc4_out);
  esc1_out = esc2_out = esc3_out = esc4_out = 0;

  /*Serial.printf("pitch p:%.2f,i:%.2f,d:%.2f,out:%.2f", pitch_pid.p, pitch_pid.i, pitch_pid.d, pid_p);
  Serial.printf("-roll p:%.2f,i:%.2f,d:%.2f,out:%.2f\n", roll_pid.p, roll_pid.i, roll_pid.d, pid_r);
  Serial.printf("-Esc1:%d Esc2:%d,Esc3:%d,Esc4:%d\n", esc1_out, esc2_out, esc3_out, esc4_out);*/
}

void FC::MotionUpdate(){
  mpu.update();
  gyro[0]     = mpu.getAngleX();
  gyro[1]     = mpu.getAngleY();
  gyro[2]     = mpu.getAngleZ();

  accel[0]    = mpu.getAccX();
  accel[1]    = mpu.getAccY();
  accel[2]    = mpu.getAccZ();
}

void FC::writeEsc(uint32_t esc1, uint32_t esc2, uint32_t esc3, uint32_t esc4){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, esc3);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, esc4);
}

void FC::parseCommand(){
  
}

void FC::disarm(){
  for(int i = 0; i<200;i++){ // Force rotor stop? weird bug!!
  if(i==0){armed = 0;}
  writeEsc(0,0,0,0);
  }
}
