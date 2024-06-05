#include "drivers/receiver/pwm.hpp"
#include "esp_log.h"
#include <array>

static const char* TAG = "PWM Receiver Driver";

PWMReceiver::PWMReceiver() {
    type = RxType::PWM;
    init();
}

void PWMReceiver::init() {
    ESP_LOGI(TAG, "Initializing RMT Rx channels");

    rmt_channel_handle_t channels[PWM_CHANNELS_NUM];

    for (uint8_t i = 0; i < PWM_CHANNELS_NUM; i++) {
        // Configure the RMT Rx channel
        rmt_rx_channel_config_t rmt_rx = {
            .gpio_num = RECEIVER_GPIO[i],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz resolution
            .mem_block_symbols = 64,
            .flags = {.with_dma = false},
        };

        // Initialize the RMT Rx channel
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx, &channels[i]));

        // RX configuration
        rmt_receive_config_t cfg = {
            // The PWM range is typically 1000 to 2000 microseconds for standard RC receivers.
            // Converting these values to nanoseconds because the ESP32 RMT configuration requires it in nanoseconds.
            .signal_range_min_ns = 1000 * 1000,  // Minimum pulse width: 1000 µs = 1,000,000 ns
            .signal_range_max_ns = 2000 * 1000,  // Maximum pulse width: 2000 µs = 2,000,000 ns
        };


        // Register the read callback
        isr_args ctx = isr_args{.target = &output[i]};
        rmt_rx_register_event_callbacks(channels[i], &rmt_callbacks, &ctx);

        // Enable the RMT Rx channel and start receiving
        ESP_ERROR_CHECK(rmt_enable(channels[i]));
        ESP_ERROR_CHECK(rmt_receive(channels[i], &values[i], sizeof(values[i]), &cfg)); // TODO: move to separate function
    }

    ESP_LOGI(TAG, "RMT Rx channels initialized");
}