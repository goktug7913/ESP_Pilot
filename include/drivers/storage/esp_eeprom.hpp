#include "drivers/storage/base.hpp"

class Eeprom : public StorageDriver {
public:
    Eeprom();
    ~Eeprom();
    void init() override;
    ESP_Pilot_Config readConfig() override;
    void writeConfig(ESP_Pilot_Config config) override;
};