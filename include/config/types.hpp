#pragma once
#include "control/pid.hpp"
#include "control/input.hpp"
#include "drivers/receiver/base.hpp"

struct WiFi_Config {
    char ssid[32] = "";
    char password[64] = "";
};

enum MotorGeometry {
    QUAD_X,
    QUAD_PLUS,
};

struct ESP_Pilot_Config {
    MotorGeometry motorGeometry;
    InputLayout inputLayout;
    WiFi_Config wifi;
    PIDCoefficients pidGainsPitch;
    PIDCoefficients pidGainsRoll;
    PIDCoefficients pidGainsYaw;
    RxType receiverType;
};