#include "drivers/receiver/pwm.hpp"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include <array>

static const char* TAG = "PWM Receiver Driver";

constexpr std::array<uint8_t, 6> RECEIVER_GPIO = {35, 34, 39, 36, 27, 14};
constexpr int PWM_CHANNELS_NUM = 6;                                                // Receiver PWM Channel Number
constexpr std::array<uint8_t, PWM_CHANNELS_NUM> PWM_CHANNELS = {1, 2, 3, 4, 5, 6}; // Change the channels according to the number of channels

PWMReceiver::PWMReceiver() {
    type = RxType::PWM;
    init();
}

void PWMReceiver::init() {
    ESP_LOGI(TAG, "Initializing RMT Rx channels");

    rmt_channel_handle_t channels[PWM_CHANNELS_NUM];

    for (size_t i = 0; i < PWM_CHANNELS_NUM; i++) {
        rmt_rx_channel_config_t rmt_rx = {
            .gpio_num = RECEIVER_GPIO[i],
            .clk_src = rmt_clock_source_t::RMT_CLK_SRC_APB,
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz resolution
            .mem_block_symbols = 64,
            .flags = {.with_dma = false},
        };

        ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx, &channels[i]));

        rmt_receive_config_t cfg = {
            .signal_range_min_ns = 1000,
            .signal_range_max_ns = 2000,
        };

        ESP_ERROR_CHECK(rmt_enable(channels[i]));
        ESP_ERROR_CHECK(rmt_receive(channels[i], &stub, sizeof(stub), &cfg));
    }
    ESP_LOGI(TAG, "RMT Rx channels initialized");
}

void PWMReceiver::read() {
}
  