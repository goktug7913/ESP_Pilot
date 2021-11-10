#include "ConfigSuite.h"

extern FC_cfg cfg;

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
    Serial.write(W_EEPROM_ERR);
  }
}

void ConfigSuite::receiveCfg() {
  //byte w_data[sizeof(FC_cfg)];  // Buffer containing the bytes to write to the EEPROM
  Serial.read((uint8_t*)&current_config, sizeof(FC_cfg));

  //memcpy(&current_config, &w_data, sizeof(FC_cfg));  //Load the received config data
  Serial.write((byte*)&current_config, sizeof(FC_cfg)); // REMOVE LATER bounce back received data

  Serial.printf("Received cfg with header: %08X", current_config.header); // REMOVE LATER
}

bool ConfigSuite::writeCfg() {
  byte w_data[sizeof(FC_cfg)];
  memcpy(&w_data, &current_config, sizeof(FC_cfg));

  if (current_config.header == CFG_MAGIC) {
    for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) {EEPROM.write(i, w_data[i]);}
    EEPROM.commit();
    Serial.println("\nWrite cfg finished"); // REMOVE LATER
  }

  // CHECK IF FLASHED CORRECTLY
  FC_cfg candidate_cfg;         // Candidate Config data read back
  byte r_data[sizeof(FC_cfg)];  // Buffer containing the bytes read from the EEPROM

  for (int i = EEPROM_START_ADDR; i < sizeof(FC_cfg); i++) { r_data[i] = EEPROM.read(i); }

  memcpy(&candidate_cfg, &r_data, sizeof(FC_cfg));  //Load back the candidate configuration

  if (memcmp(&candidate_cfg, &current_config, sizeof(FC_cfg)) == 0) {
    applyDraftCfg();
    Serial.write(W_EEPROM_OK);
    Serial.println("\nconfig header ok!"); // REMOVE LATER
    return 1;
  } else {
    Serial.write(W_EEPROM_ERR);
    Serial.println("\nconfig header corrupted!"); // REMOVE LATER
    return 0;
  }
}

FC_cfg* ConfigSuite::requestConfig(){
  return &current_config;
}
