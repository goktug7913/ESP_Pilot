#include "SerialManager.h"
#include "Controller.h"
#include "Telemetry.h"
#include "Config.h"

extern FC FliCon;
extern FC_cfg cfg;
extern ConfigSuite CfgMan;
extern TelemetryManager Logger;

void SerialMgr::SendMsg(uint8_t code){
  //Overload for sending only a command code
  SendMsg(code, 0, nullptr);
}

void SerialMgr::SendMsg(uint8_t code, uint16_t opt){
  //Overload without data
  SendMsg(code, opt, nullptr);
}

void SerialMgr::SendMsg(uint8_t code, uint16_t opt, uint8_t* data){
  //Base message sending function
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

void SerialMgr::ReceiveMsg(){
  // Starts reading serial bytes until it finds a the start of a message
  data_seek();
}

void SerialMgr::data_seek(){
  //Find the Message Start flag
  while(Serial.available() > 0 && state == seek){
    inb = Serial.read();
    if (inb == MSG_START){
      recvbytes[bx] = inb;
      bx++;
      state = reading;
      break;
      }
      else if(inb == MSG_END){
        //If you're here, reading started mid-packet
        memset(&recvbytes, 0, bx);
        bx = 0;
      }
  }
  if (state == reading){data_read();} //Start reading the message
}

void SerialMgr::data_read(){
  //Read data until Message End flag
  while(Serial.available() > 0){
      inb = Serial.read();
      recvbytes[bx] = inb;
      bx++;
      if (inb == MSG_END){
        state = done;
        break;
      }
  }
  if (state == done){data_done();} //Process the message
}

void SerialMgr::data_done(){
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
      FliCon.usbmode = true; //Set USB mode
      SendMsg(HANDSHAKE); //Send handshake
    break;
    // - - - - - - - - - - - - - - - - -
    case READ_CFG:
      SendMsg(CFG_DATA_FLAG,0,(uint8_t*)CfgMan.getActiveCfg()); //Send config data to configurator
    break;
      // - - - - - - - - - - - - - - - - -
    case WRITE_CFG:
      CfgMan.setCfg((uint8_t*)&recvdata); //Receive config data from configurator
      CfgMan.setFlashCfg((FC_cfg*)&recvdata); //Callback is inside this func
    break;
    // - - - - - - - - - - - - - - - - -
    case ARM_TETHERED:
      FliCon.armed = true; //Set armed flag
    break;
    // - - - - - - - - - - - - - - - - -
    case DISARM:
      FliCon.disarm(); //Disarm the FC
    break;
    // - - - - - - - - - - - - - - - - -
    case RESTART_FC:
      ESP.restart(); //Restart the FC
    break;
    // - - - - - - - - - - - - - - - - -
    case TELEMETRY_START:
      Logger.StartSerial(); //Start the telemetry stream
    break;
    // - - - - - - - - - - - - - - - - -
    case TELEMETRY_STOP:
      Logger.StopSerial(); //Stop the telemetry stream
    break;
    // - - - - - - - - - - - - - - - - -
    case SET_PID_GAIN:     // Dev note: this is a special case, since it's not a normal message
      float p,i,d;
      p = *(float*)&header.opt;
      i = *((float*)&header.opt+1);
      d = *((float*)&header.opt+2);
      FliCon.setGains(p,i,d);
    break;
    // - - - - - - - - - - - - - - - - -
  }
  state = seek;
}