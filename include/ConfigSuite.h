#include "definitions.h"
//#include <EEPROM.h>
#include "Config.h"
//#include "Arduino.h"
#pragma once

class ConfigSuite{
  public:

  int eeprom_size = EEPROM_SIZE;        // size of the EEPROM in bytes

  FC_cfg  getFlashCfg();                //Reads the config from the flash memory
  bool    setFlashCfg(FC_cfg* cfg);     //Writes the config to the flash memory

  void    setCfg(FC_cfg* cfg);          //set active config from cfg struct
  void    setCfg(uint8_t* cfgbytes);    //set active config from byte array
  FC_cfg* getActiveCfg();               //returns current cfg (not in eeprom)


  private:

  FC_cfg current_config;               //current config

  bool   isValidCfg(FC_cfg* cfg);      //checks header
  FC_cfg bytetoCfg(uint8_t* cfgbytes); //convert byte array to config
};