#pragma once
#include "config/types.hpp"
#include "drivers/storage/base.hpp"

class ConfigManager {
private:
    ESP_Pilot_Config config;
    StorageDriver* storage;
public:
    ConfigManager();
    ~ConfigManager();
};