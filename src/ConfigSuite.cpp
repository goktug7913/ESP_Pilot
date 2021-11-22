#include "ConfigSuite.h"
#include "SerialManager.h"

//extern FC_cfg cfg;
extern SerialMgr SerialMan;

FC_cfg ConfigSuite::getFlashCfg(){
  FC_cfg flashcfg;
  uint8_t readbytes[sizeof(FC_cfg)]; //Temporary buffer to read from flash

  //Read from flash
  for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++){
    readbytes[i] = EEPROM.read(i);
  }
  memcpy(&flashcfg, readbytes, sizeof(FC_cfg)); //Copy the read bytes to the flashcfg struct

  return flashcfg;
}

void ConfigSuite::setCfg(uint8_t* cfgbytes){
  memcpy(&current_config, &cfgbytes, sizeof(FC_cfg));  //Load the received config data
}

bool ConfigSuite::setFlashCfg(FC_cfg* cfg){
  bool status = false;

  if (isValidCfg(cfg)) {
    for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) {EEPROM.write(i, *(uint8_t*)&cfg+i);}
    EEPROM.commit();
    SerialMan.SendMsg(W_EEPROM_OK); //Flash write successful message
  } else {
    SerialMan.SendMsg(W_EEPROM_ERR); //Flash write failed message
  }

  return status;
}

FC_cfg* ConfigSuite::getActiveCfg(){
  return &current_config;
}

bool ConfigSuite::isValidCfg(FC_cfg* cfg) {
  bool status = false;
  if (cfg->header == CFG_MAGIC && cfg->footer == CFG_MAGIC2) {status = true;}
  return status;
}

FC_cfg ConfigSuite::bytetoCfg(uint8_t* cfgbytes){
  FC_cfg cfg;
  memcpy(&cfg, cfgbytes, sizeof(FC_cfg));

  if (isValidCfg(&cfg)) {
    return cfg;
  } else {
    return current_config; //Return the current config if the received config is invalid, need to check this
  }
}
