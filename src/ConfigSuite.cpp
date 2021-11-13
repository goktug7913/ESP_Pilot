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

void ConfigSuite::receiveCfg(uint8_t* cfgbytes){
  memcpy(&current_config, &cfgbytes, sizeof(FC_cfg));  //Load the received config data
}

bool ConfigSuite::setFlashCfg(FC_cfg* cfg){
  //byte w_data[sizeof(FC_cfg)];
  //memcpy(&w_data, &current_config, sizeof(FC_cfg));
  bool status = false;

  if (cfg->header == CFG_MAGIC) {
    for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) {EEPROM.write(i, *(uint8_t*)&cfg+i);}
    EEPROM.commit();
    //SerialMan.SendMsg(W_EEPROM_OK); //Flash write successful message
  }

  return status;
}

FC_cfg* ConfigSuite::getActiveCfg(){
  return &current_config;
}

bool isValidCfg(FC_cfg* cfg) {
  bool status = false;
  if (cfg->header == CFG_MAGIC) {
    status = true;
  }
  return status;
}