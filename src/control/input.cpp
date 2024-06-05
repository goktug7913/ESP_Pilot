#include "control/input.hpp"
#include "drivers/receiver/pwm.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "InputSystem";

InputSystem::InputSystem() {
    init();
}

InputSystem::InputSystem(InputLayout l) {
    this->layout = l;

    // Determine the correct receiver class to construct.
    switch (layout.rx_type) {
    case RxType::PWM:
        rx_driver = new PWMReceiver();
        break;
    default:
        ESP_LOGE(TAG, "Invalid receiver type");
        break;
    }

    // Continue init
    init();
}

/**
 * Initialize the driver
*/
void InputSystem::init() {
    rx_driver = new PWMReceiver();

    // Spawn a task to read the receiver values
    BaseType_t taskCreated;
    taskCreated = xTaskCreate([](void* obj) {
        InputSystem* input = (InputSystem*)obj;
        while (true) {
            input->read();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }, "InputTask", 2048, this, 5, NULL);

    if(taskCreated == pdPASS) {
        ESP_LOGI(TAG, "Task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create task");
    }
}

/**
 * Read the receiver values
*/
void InputSystem::read() {
    // Read the receiver
    for (int i = 0; i < layout.num_channels; i++) {
        Channel c = layout.channels[i];
        c.setInput(rx_driver->getChannel(i));

        // Log the channel value
        ESP_LOGI(TAG, "Channel %d: %f", i, c.getOutput());
    }
}