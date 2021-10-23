#include "definitions.h"
#include "stdint.h"
#pragma once

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------SYNC ANY CHANGES TO THIS STRUCT WITH ESP_CONFIGURATOR APP CODE --------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
const static uint8_t    pwm_ch_amt = PWM_CHANNELS_NUM;

typedef struct{
  uint32_t          header = CFG_MAGIC;

  //Control
  float             p_gain = 25;
  float             i_gain = 1;
  float             d_gain = 1;
  uint8_t           esc_pwm_hz = PWM_FREQ;
  uint8_t           max_angle = 20;

  //Settings
  float             vBat = 12.6;
  bool              vBat_compensation = 0;

  //Aux modules
  bool              nrf24_telemetry = 0;
  bool              oled_display = 0;
  bool              radar_altimeter = 0;
  bool              compass = 0;

  //ESC signal pins (initialized as defined in code)
  uint8_t           esc1_pin = ESC_1;
  uint8_t           esc2_pin = ESC_2;
  uint8_t           esc3_pin = ESC_3;
  uint8_t           esc4_pin = ESC_4;

  //PWM Radio
  volatile uint16_t       ReceiverChannels[pwm_ch_amt]    = {0};
  uint8_t                 RECEIVER_CHANNELS[pwm_ch_amt]   = {PWM_CHANNELS};
  uint8_t                 RECEIVER_GPIOS[pwm_ch_amt]      = {RECEIVER_GPIO};
  uint8_t                 rx_deadzone = RX_DEADZONE;

  //uint32_t          footer = CFG_MAGIC2;

} FC_cfg;