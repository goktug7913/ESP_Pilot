#include "SerialManager.h"
#include "Controller.h"
#include "Telemetry.h"

extern FC FliCon;
extern FC_cfg cfg;
extern ConfigSuite CfgMan;
extern TelemetryManager Logger;

void SerialMgr::ReceiveMsg(){
  while(Serial.available() > 0){
  switch(state){

    case seek:
    //Find the Message Start flag
    while(Serial.available() > 0){
      inb = Serial.read();
      if (inb == MSG_START){
        recvbytes[bx] = inb;
        bx++;
        state = reading;
        return;
      }else if(inb == MSG_END){
        //If you're here, reading started mid-packet
        memset(&recvbytes, 0, bx);
        bx = 0;
      }
    }
    break;

    case reading:
    //Read data until Message End flag
    while(Serial.available() > 0){
      inb = Serial.read();
      recvbytes[bx] = inb;
      bx++;
      if (inb == MSG_END){
        state = done;
        return;
      }
    }
    break;

    case done:
    msg_begin header;
    msg_end footer;
    rxlen = bx;
    bx = 0;

    memcpy(&header, &recvbytes, sizeof(header));
    memcpy(&footer, &recvbytes+sizeof(recvbytes)-sizeof(footer), sizeof(footer));
    
    //extract data here...
    //since sizeof is compile time, we need to set the data length accordingly
    uint16_t datasize = 0;
    if (header.cmd == WRITE_CFG){datasize = sizeof(FC_cfg);}

    memcpy(&recvdata, &recvbytes+sizeof(header), datasize);

    //Zero fill buffer
    memset(&recvbytes, 0, rxlen);

    switch (header.cmd){
      // - - - - - - - - - - - - - - - - -
      case CFG_MODE:
        FliCon.usbmode = 1;
        SendMsg(HANDSHAKE);
      break;
      // - - - - - - - - - - - - - - - - -
      case READ_CFG:
        SendMsg(CFG_DATA_FLAG,0,(uint8_t*)CfgMan.requestConfig());
      break;
      // - - - - - - - - - - - - - - - - -
      case WRITE_CFG:
        CfgMan.receiveCfg((uint8_t*)&recvdata);
        CfgMan.writeCfg(); //Callback is inside this func
        CfgMan.applyDraftCfg();
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
    state = seek;
    break;
  }
  }
}

void SerialMgr::SendMsg(uint8_t code){
  SendMsg(code, 0, nullptr);
}

void SerialMgr::SendMsg(uint8_t code, uint16_t opt){
  SendMsg(code, opt, nullptr);
}

void SerialMgr::SendMsg(uint8_t code, uint16_t opt, uint8_t* data){
  msg_begin header;
  msg_end footer;

  //since sizeof is compile time, we need to set the data length accordingly
  uint16_t datasize = 0;
  if (code == CFG_DATA_FLAG){datasize = sizeof(FC_cfg);}
  else if (code == TMTY_DATA_FLAG){datasize = sizeof(telemetry_frame);}

  header.cmd = code;
  header.opt = opt;
  header.length = sizeof(header)+datasize+sizeof(footer);
  header.start = MSG_START;
  header.data_start = DATA_START;

  footer.data_end = DATA_END;
  footer.end = MSG_END;

  Serial.write((uint8_t*)&header,sizeof(header));
  // Place binary data between header-footer if there is any data for this msg
  if (data != nullptr){Serial.write(data, datasize);}
  Serial.write((uint8_t*)&footer,sizeof(footer));
}