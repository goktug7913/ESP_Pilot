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

  void loadCfg();

  void receiveCfg();

  bool writeCfg();

  private:
  FC_cfg current_config;
};