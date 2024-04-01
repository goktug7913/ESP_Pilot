#pragma once
#include "control/pid.hpp"

struct WiFi_Config {
    char ssid[32] = "";
    char password[64] = "";
};

struct ESP_Pilot_Config {
    WiFi_Config wifi;
    PIDCoefficients pidGainsPitch;
    PIDCoefficients pidGainsRoll;
    PIDCoefficients pidGainsYaw;
};