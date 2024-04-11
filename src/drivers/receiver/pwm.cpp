#include "drivers/receiver/pwm.hpp"
#include "driver/rmt.h"
#include <array>

constexpr std::array<uint8_t, 6> RECEIVER_GPIO = {35, 34, 39, 36, 27, 14};
constexpr int PWM_CENTER = 1500; // Center of receiver PWM pulse
constexpr int RX_DEADZONE = 2;   // Deadzone for receiver PWM pulse

constexpr int RMT_TICK_PER_US = 1;                                                 // determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
constexpr int RMT_RX_MAX_US = 3500;                                                // time before receiver goes idle (longer pulses will be ignored)
constexpr int PWM_CHANNELS_NUM = 6;                                                // Receiver PWM Channel Number
constexpr std::array<uint8_t, PWM_CHANNELS_NUM> PWM_CHANNELS = {1, 2, 3, 4, 5, 6}; // Change the channels according to the number of channels
constexpr int RMT_RX_CLK_DIV = (80000000 / RMT_TICK_PER_US / 1000000); // Divide 80Mhz APB clk by 1Mhz to get the clock for the receiver

PWMReceiver::PWMReceiver() {
    type = RxType::PWM;
    init();
}

void PWMReceiver::init() {
  