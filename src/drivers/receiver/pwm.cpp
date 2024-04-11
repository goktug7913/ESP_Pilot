#include "drivers/receiver/pwm.hpp"

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
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_CHANNEL_0;
    rmt_rx.gpio_num = (gpio_num_t)RECEIVER_GPIO[0];
    rmt_rx.clk_div = RMT_RX_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
    rmt_get_ringbuf_handle(rmt_rx.channel, &rb);
    rmt_rx_start(rmt_rx.channel, true);
}

void PWMReceiver::read() {
    size_t rx_size;
    rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 1000);
    if (items) {
        for (size_t i = 0; i < rx_size / 4; i++) {
            uint16_t value = items[i].duration0;
            if (value > PWM_CENTER - RX_DEADZONE && value < PWM_CENTER + RX_DEADZONE) {
                setChannel(PWM_CHANNELS[i], PWM_CENTER);
            } else {
                setChannel(PWM_CHANNELS[i], value);
            }
        }
        vRingbufferReturnItem(rb, (void*)items);
    }
}
  