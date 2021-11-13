#include "Controller.h"
#include "Telemetry.h"
#include "SerialManager.h"

//extern FC_cfg cfg; // TODO: deprecate
extern ConfigSuite CfgMan;
extern TelemetryManager Logger;
extern MPU6050 mpu;
extern SerialMgr SerialMan;

FC::FC(){
}

void FC::Start(){ //

  // Create PID Controller objects in the constructor
  pitch_pid = PID(CfgMan.getActiveCfg()->Kp_pitch, CfgMan.getActiveCfg()->Ki_pitch, CfgMan.getActiveCfg()->Kd_pitch);
  roll_pid = PID(CfgMan.getActiveCfg()->Kp_roll, CfgMan.getActiveCfg()->Ki_roll, CfgMan.getActiveCfg()->Kd_roll);
  yaw_pid = PID(CfgMan.getActiveCfg()->Kp_yaw, CfgMan.getActiveCfg()->Ki_yaw, CfgMan.getActiveCfg()->Kd_yaw);
  //alt_pid = PID(CfgMan.getActiveCfg()->Kp_alt, CfgMan.getActiveCfg()->Ki_alt, CfgMan.getActiveCfg()->Kd_alt);
  
  uint8_t tCtr = 0; // Counter for the telemetry

  while(armed){ // Loop while armed
    if(rx_raw[4] < 1600 && rx_raw[4] > 1400){writeEsc(rx_raw[2], rx_raw[2], rx_raw[2], rx_raw[2]);} // Bypass PID when SWC is pos2, for ESC signal calibration
    tCtr++;
    SerialMan.ReceiveMsg(); // Receive serial data
    MotionUpdate(); // Update motion data
    InputTransform(); // Transform input data

    pid_p = pitch_pid.Calculate( gyro[0], rx_scaled[1] ); // Calculate pitch PID output
    pid_r =  roll_pid.Calculate( gyro[1], rx_scaled[0] ); // Calculate roll PID output
    pid_y =   yaw_pid.Calculate( gyro[2], rx_scaled[2] ); // Calculate yaw PID output

    // Brickwall PID limiter to prevent saturation
    if (pid_p > 1800){pid_p = 1800;} else if (pid_p < -1800){pid_p = -1800;} // Pitch
    if (pid_r > 1800){pid_r = 1800;} else if (pid_r < -1800){pid_r = -1800;} // Roll

    OutputTransform(); // Transform the output to the ESCs
    
    if (rx_raw[4] < 1250){armed = 0;}  // Disarm if CH5 low
    if (Logger.enableserial && tCtr == 35){Logger.SerialSendFrame(); tCtr = 0;} // Send telemetry over serial when activated
  }
}

void FC::InputTransform(){
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP ROLL SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (rx_raw[3] >= PWM_CENTER+CfgMan.getActiveCfg()->rx_deadzone){ // Stick Right
    rx_scaled[0] = map( rx_raw[3],  PWM_CENTER,  2000,  0,  CfgMan.getActiveCfg()->max_angle ); 
  }
  else if (rx_raw[3] < PWM_CENTER-CfgMan.getActiveCfg()->rx_deadzone){ // Stick Left
    rx_scaled[0] = 0-map( rx_raw[3],  PWM_CENTER,  1000,  0,  CfgMan.getActiveCfg()->max_angle );
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  // MAP PITCH SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (rx_raw[1] >= PWM_CENTER+CfgMan.getActiveCfg()->rx_deadzone){ // Stick Up
    rx_scaled[1] = map( rx_raw[1],  PWM_CENTER,  2000,  0,  CfgMan.getActiveCfg()->max_angle );
  }
  else if (rx_raw[1] < PWM_CENTER-CfgMan.getActiveCfg()->rx_deadzone){ // Stick Down
    rx_scaled[1] = 0-map( rx_raw[1],  PWM_CENTER,  1000,  0,  CfgMan.getActiveCfg()->max_angle );
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // MAP YAW SIGNAL TO DEGREE TARGET
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //Yaw target angle is increased or decreased by moving the stick, which
  //does not return to zero, it is kept like the last value instead.

  //The MPU library measures TOTAL yaw rotation angle in both directions,
  //so the angle target can be over 360 or below 0 degrees.

  //The rate of change varies by execution speed, that needs to be fixed.

  if (rx_raw[0] >= PWM_CENTER+CfgMan.getActiveCfg()->rx_deadzone){ // Yaw Stick Right
    rx_scaled[2] += map( rx_raw[0],  PWM_CENTER,  2000,  0,  CfgMan.getActiveCfg()->max_angle )/100; // Reduced speed by dividing
  }
  else if (rx_raw[0] < PWM_CENTER-CfgMan.getActiveCfg()->rx_deadzone){ // Yaw Stick Left
    rx_scaled[2] -= map( rx_raw[0],  PWM_CENTER,  1000,  0,  CfgMan.getActiveCfg()->max_angle )/100; // Reduced speed by dividing
  }
}

void FC::OutputTransform(){

  esc1_out = esc2_out = esc3_out = esc4_out = 0; // Reset ESCs to 0
  
  // Calculate Roll PWM
  if (pid_r > 0){ // Positive Roll
    esc1_out += pid_r;  esc2_out -= pid_r;
    esc3_out += pid_r;  esc4_out -= pid_r;
  } else { // Negative Roll
    esc2_out -= pid_r;  esc1_out += pid_r;
    esc4_out -= pid_r;  esc3_out += pid_r;
  }

  // Calculate Pitch PWM
  if (pid_p < 0){ // Negative Pitch
    esc1_out -= pid_p;  esc3_out += pid_p;
    esc2_out -= pid_p;  esc4_out += pid_p;
  } else { // Positive Pitch
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
}

void FC::MotionUpdate(){
  mpu.update(); 
  gyro[0]     = mpu.getAngleX(); // Roll
  gyro[1]     = mpu.getAngleY(); // Pitch
  gyro[2]     = mpu.getAngleZ(); // Yaw

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

void FC::disarm(){
  for(int i = 0; i<200;i++){ // Force rotor stop? weird bug!!
  if(i==0){armed = 0;} 
  writeEsc(0,0,0,0);
  }
}
