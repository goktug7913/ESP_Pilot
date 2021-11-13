#include "definitions.h"
#include <EEPROM.h>
#include "Config.h"
#include "Arduino.h"
#pragma once

class ConfigSuite{
  public:
  int eeprom_size = EEPROM_SIZE;

  void sendCfg();

  void applyDraftCfg();

  void loadCfg(); // get eeprom cfg

  void receiveCfg(uint8_t* cfgbytes);

  bool writeCfg();

  FC_cfg* requestConfig(); //returns current cfg (not in eeprom)

  private:
  FC_cfg current_config;
};