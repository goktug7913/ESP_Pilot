#include "definitions.h"
#include <cstdint>
#pragma once

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------SYNC ANY CHANGES TO THIS STRUCT WITH ESP_CONFIGURATOR APP CODE --------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
const static uint8_t    pwm_ch_amt = PWM_CHANNELS_NUM;          // Number of PWM channels

struct FC_cfg{
  uint32_t          header = CFG_MAGIC;                         // Magic number to identify config data

  //Control
  float             Kp_pitch = 2;                               // Proportional gain for pitch
  float             Ki_pitch = 0.0010;                           // Integral gain for pitch
  float             Kd_pitch = 12;                              // Derivative gain for pitch

  float             Kp_roll = 0.67;                                // Proportional gain for roll
  float             Ki_roll = 0.025;                                // Integral gain for roll
  float             Kd_roll = 1280;                               // Derivative gain for roll

  float             Kp_yaw = 3;                                 // Proportional gain for yaw
  float             Ki_yaw = 0.001;                             // Integral gain for yaw
  float             Kd_yaw = 0;                                 // Derivative gain for yaw

  uint8_t           esc_pwm_hz = PWM_FREQ;                      // ESC PWM frequency in Hz
  uint8_t           max_angle = 15;                              // Max angle in degrees

  //Settings
  float             vBat = 11.1;                                // Battery max voltage
  bool              vBat_compensation = false;                      // Enable battery voltage compensation

  //Aux modules
  bool              nrf24_telemetry = false;                        // Enable nrf24 telemetry
  bool              radar_altimeter = false;                        // Enable radar altimeter
  bool              compass = false;                                // Enable compass

  //ESC signal pins (initialized as defined in code)
  uint8_t           esc1_pin = ESC_1;
  uint8_t           esc2_pin = ESC_2;
  uint8_t           esc3_pin = ESC_3;
  uint8_t           esc4_pin = ESC_4;

  //PWM Radio
  volatile uint16_t ReceiverChannels[pwm_ch_amt]    = {0};
  uint8_t           RECEIVER_CHANNELS[pwm_ch_amt]   = {PWM_CHANNELS};
  uint8_t           RECEIVER_GPIOS[pwm_ch_amt]      = {RECEIVER_GPIO};
  uint8_t           rx_deadzone = RX_DEADZONE;                  // Deadzone for receiver
  uint32_t footer = CFG_MAGIC2;                                 // Magic number to identify config data
};