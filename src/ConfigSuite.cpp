#include "ConfigSuite.h"
#include "SerialManager.h"

extern FC_cfg cfg;
extern SerialMgr SerialMan;
void ConfigSuite::sendCfg() {
  Serial.write((byte*)&current_config, sizeof(FC_cfg));  //weird casting here lol
}

void ConfigSuite::applyDraftCfg() {
  memcpy(&cfg, &current_config, sizeof(FC_cfg));
}

void ConfigSuite::loadCfg() {
  FC_cfg candidate_cfg;       // Temporary config struct object for checking the magic header
  byte data[sizeof(FC_cfg)];  // Buffer containing the bytes read from the EEPROM

  for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) { data[i] = EEPROM.read(i); }

  memcpy(&candidate_cfg, &data, sizeof(FC_cfg));  //Load the candidate configuration to draft configuration

  if (candidate_cfg.header == CFG_MAGIC) {
    memcpy(&current_config, &candidate_cfg, sizeof(FC_cfg));
  } else {
    SerialMan.SendMsg(W_EEPROM_ERR);
  }
}

void ConfigSuite::receiveCfg(uint8_t* cfgbytes) {
  memcpy(&current_config, &cfgbytes, sizeof(FC_cfg));  //Load the received config data
}

bool ConfigSuite::writeCfg() {
  //byte w_data[sizeof(FC_cfg)];
  //memcpy(&w_data, &current_config, sizeof(FC_cfg));

  if (current_config.header == CFG_MAGIC) {
    for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) {EEPROM.write(i, *(uint8_t*)&current_config+i);}
    EEPROM.commit();
    SerialMan.SendMsg(W_EEPROM_OK);
  }

  // CHECK IF FLASHED CORRECTLY
  FC_cfg candidate_cfg;         // Candidate Config data read back
  byte r_data[sizeof(FC_cfg)];  // Buffer containing the bytes read from the EEPROM

  for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) { r_data[i] = EEPROM.read(i); }

  memcpy(&candidate_cfg, &r_data, sizeof(FC_cfg));  //Load back the candidate configuration

  if (memcmp(&candidate_cfg, &current_config, sizeof(FC_cfg)) == 0) {
    applyDraftCfg();
    Serial.write(W_EEPROM_OK);
    return 1;
  } else {
    Serial.write(W_EEPROM_ERR);
    return 0;
  }
}

FC_cfg* ConfigSuite::requestConfig(){
  return &current_config;
}
