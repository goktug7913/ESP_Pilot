#include "config/config_manager.hpp"

static const char* TAG = "ConfigManager";

ConfigManager::ConfigManager() {
    // Load default values
}

ConfigManager::~ConfigManager() {
    // Save config?
}

ESP_Pilot_Config ConfigManager::getConfig() {
    return config;
}

void ConfigManager::setConfig(ESP_Pilot_Config config) {
    this->config = config;
}

bool ConfigManager::saveConfig() {
    return false;
}

bool ConfigManager::loadConfig() {
    return false;
}