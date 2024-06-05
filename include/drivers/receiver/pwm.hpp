#pragma once
#include "drivers/receiver/base.hpp"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <array>

struct isr_args {
    uint16_t* target;
};

class PWMReceiver : public virtual ReceiverBase {
public:
    PWMReceiver();
    ~PWMReceiver() override = default;
    void init() override;
    uint16_t getChannel(uint8_t channel) { return output[channel]; };
    std::array<uint16_t, 6> output = {1500, 1500, 1500, 1500, 1500, 1500};

private:
    int PWM_CHANNELS_NUM = 6; // Receiver PWM Channel Number
    std::array<uint8_t, 6> RECEIVER_GPIO = {35, 34, 39, 36, 27, 14}; // We need to un-hardcode these
    //std::array<uint8_t, 6> PWM_CHANNELS = {1, 2, 3, 4, 5, 6}; // TODO: Why did this get carried over from Arduino branch?
    std::array<rmt_symbol_word_t[64], 6> values = {1500, 1500, 1500, 1500, 1500, 1500}; // Receiver PWM values in microseconds
    
    // Rx callback API wrapper
    rmt_rx_event_callbacks_t rmt_callbacks = {
        .on_recv_done = &read_callback,
    };

    // Static callback handler compatible with rmt_rx_done_callback_t
    static bool read_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
        assert(user_data != nullptr);
        assert(edata != nullptr);

        auto ctx = reinterpret_cast<isr_args*>(user_data);
  
        if (edata->num_symbols > 0) {
            auto sym = edata->received_symbols;
            // Process each symbol's pulse width
            for (size_t i = 0; i < edata->num_symbols; i++) {
                if (i % 2 == 0) { // Assuming even index for high pulse
                    uint32_t pulse_width = sym[i].duration0;
                    *ctx->target = pulse_width / 80; // Convert from 80 MHz clock cycles to microseconds
                }
            }
        }

        return true; // Indicate that the event has been handled
    }
};