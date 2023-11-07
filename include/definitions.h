#pragma once

#include <array>

constexpr bool MPU_UPSIDEDOWN = true; // Enable if MPU chip faces downward (default is facing up)

constexpr int PWM_FREQ = 250;      // ESC Control signal, Default: 50Hz
constexpr int PWM_MIN_DUTY = 1000; // ESC Minimum Pulse Width (us)
constexpr int PWM_MAX_DUTY = 2000; // ESC Maximum Pulse Width (us)

constexpr std::array<int, 4> ESC_PINS = {26, 25, 33, 32}; // PWM Pins

constexpr float ITERMSCALAR = 1;     // Integral term scalar (TESTING)
constexpr int ITERMLIMIT = 1280;     // Integral term limit (TESTING)
constexpr float ITERMDEADBAND = 0.5; // Integral term deadband degrees (TESTING)
constexpr int PIDLIMIT = 475;        // PID PWM output limit (TESTING)
constexpr float PIDMASTERGAIN = 1.0; // PID master gain (TESTING)

constexpr int RF24_CE = 2;         // nRF24 CE Pin
constexpr int RF24_CSN = 0;        // nRF24 CSN Pin
constexpr int RF24_FREQ = 1000000; // nRF24 SPI Speed

constexpr int EEPROM_START_ADDR = 0; // EEPROM Data start address
constexpr int EEPROM_SIZE = 512;     // EEPROM Allocated Size in Bytes (512 bytes max)
constexpr int SERIAL_BAUD = 115200;  // Serial Baud Rate

constexpr std::array<uint8_t, 6> RECEIVER_GPIO = {35, 34, 39, 36, 27, 14};
constexpr int PWM_CENTER = 1500; // Center of receiver PWM pulse
constexpr int RX_DEADZONE = 2;   // Deadzone for receiver PWM pulse

constexpr int RMT_TICK_PER_US = 1;                                                 // determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
constexpr int RMT_RX_MAX_US = 3500;                                                // time before receiver goes idle (longer pulses will be ignored)
constexpr int PWM_CHANNELS_NUM = 6;                                                // Receiver PWM Channel Number
constexpr std::array<uint8_t, PWM_CHANNELS_NUM> PWM_CHANNELS = {1, 2, 3, 4, 5, 6}; // Change the channels according to the number of channels

constexpr int RMT_RX_CLK_DIV = (80000000 / RMT_TICK_PER_US / 1000000); // Divide 80Mhz APB clk by 1Mhz to get the clock for the receiver