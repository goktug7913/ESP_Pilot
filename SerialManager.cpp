#include "SerialManager.h"
#include "Controller.h"
#include "Telemetry.h"

extern FC FliCon;
extern FC_cfg cfg;
extern ConfigSuite CfgMan;
extern TelemetryManager Logger;

void SerialMgr::ReceiveCmd(){
  if (Serial.available() > 0) {
    msg_begin header;
    msg_end footer;
    char trash[100];
  
    Serial.readBytesUntil(MSG_START, trash, 100);
    Serial.read((uint8_t*)&header, sizeof(header));

    Serial.printf("Start: %d\n", header.start);
    Serial.printf("Size: %d\n", header.length);
    Serial.printf("Cmd: %d\n", header.cmd);
    Serial.printf("DataSt: %d\n", header.data_start);

    uint8_t data[256];

    Serial.readBytesUntil(DATA_END, data, 256);
    Serial.read((uint8_t*)&footer, sizeof(footer));

    Serial.printf("DataEnd: %d\n", footer.data_end);
    Serial.printf("End: %d\n", footer.end);

    switch (header.cmd){
      // - - - - - - - - - - - - - - - - -
      case CFG_MODE:
        FliCon.usbmode = 1;
        Serial.write(HANDSHAKE);
      break;
      // - - - - - - - - - - - - - - - - -
      case READ_CFG:
        Serial.write(CFG_DATA_FLAG);
        CfgMan.loadCfg();
        CfgMan.sendCfg();
      break;
      // - - - - - - - - - - - - - - - - -
      case WRITE_CFG:
        CfgMan.receiveCfg();
        CfgMan.writeCfg();
      break;
      // - - - - - - - - - - - - - - - - -
      case ARM_TETHERED:
        FliCon.armed = 1;
      break;
      // - - - - - - - - - - - - - - - - -
      case DISARM:
        FliCon.disarm();
      break;
      // - - - - - - - - - - - - - - - - -
      case RESTART_FC:
        ESP.restart();
      break;
      // - - - - - - - - - - - - - - - - -
      case TELEMETRY_START:
        Logger.StartSerial();
      break;
      // - - - - - - - - - - - - - - - - -
      case TELEMETRY_STOP:
        Logger.StopSerial();
      break;
      // - - - - - - - - - - - - - - - - -
    }
  } else {return;}

}