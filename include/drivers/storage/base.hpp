#pragma once
#include <stddef.h>
#include "config/types.hpp"

class StorageDriver {
public:
    virtual void init() = 0;
    virtual ~StorageDriver() = default;
    virtual ESP_Pilot_Config readConfig() = 0;
    virtual void writeConfig(ESP_Pilot_Config config) = 0;
};