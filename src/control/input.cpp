#include "control/input.hpp"
#include "drivers/receiver/pwm.hpp"
#include "esp_log.h"

static const char* TAG = "InputSystem";

InputSystem::InputSystem() {
    init();
}

InputSystem::InputSystem(InputLayout l) {
    layout = l;
    switch (layout.rx_type)
    {
    case RxType::PWM:
        rx_driver = new PWMReceiver();
        break;
    default:
        ESP_LOGE(TAG, "Invalid receiver type");
        break;
    }
    init();
}

/**
 * Initialize the driver
*/
void InputSystem::init() {
    rx_driver = new PWMReceiver();
}