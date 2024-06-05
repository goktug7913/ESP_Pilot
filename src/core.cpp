#include "core.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ESP Pilot Core";

EsppCore::EsppCore() : config(ConfigManager()), input(), pidRoll(0.0f, 0.0f, 0.0f), pidPitch(0.0f, 0.0f, 0.0f), pidYaw(0.0f, 0.0f, 0.0f) {
    ESP_LOGI(TAG, "early init begin");
    this->init();
}

void EsppCore::init() {
    // Initialize system components
    // This would include setting up RTOS tasks, initializing drivers, etc.
    ESP_LOGI(TAG, "init begin");
    config.loadConfig();
    
    pidRoll.setCoefficients(1.0f, 0.0f, 0.0f);
    pidPitch.setCoefficients(1.0f, 0.0f, 0.0f);
    pidYaw.setCoefficients(1.0f, 0.0f, 0.0f);
}

void EsppCore::start() {
    // Start system components
    // This would include starting RTOS tasks, enabling drivers, etc.
    bInputEnable = true;
    bOutputEnable = true;
    bStepEnable = true;
    bSensorsEnable = true;
    ESP_LOGI(TAG, "starting");
}