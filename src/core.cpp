#include "core.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ESP Pilot Core";

EsppCore::EsppCore() {
    ESP_LOGI(TAG, "early init begin, creating objects");

    // Early initialization
    config = ConfigManager();
    input = InputSystem();
    pidRoll = PIDController(config.getConfig().pidGainsRoll);
    pidPitch = PIDController(config.getConfig().pidGainsPitch);
    pidYaw = PIDController(config.getConfig().pidGainsYaw);
    mixer = Mixer(config.getConfig().motorGeometry);

    // Handoff to init
    this->init();
}

void EsppCore::init() {
    // Initialize system components
    // This would include setting up RTOS tasks, initializing drivers, etc.
    ESP_LOGI(TAG, "init begin, loading config");
    config.loadConfig();
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