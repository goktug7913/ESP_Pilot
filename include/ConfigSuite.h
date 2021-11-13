#include "definitions.h"
#include <EEPROM.h>
#include "Config.h"
#include "Arduino.h"
#pragma once

class ConfigSuite{
  public:

  int eeprom_size = EEPROM_SIZE;

  FC_cfg  getFlashCfg();                //Reads the config from the flash memory
  bool    setFlashCfg(FC_cfg* cfg);     //Writes the config to the flash memory

  void    receiveCfg(uint8_t* cfgbytes);
  FC_cfg* getActiveCfg();               //returns current cfg (not in eeprom)

  bool    isValidCfg(FC_cfg* cfg);      //checks header

  private:

  FC_cfg current_config;
};