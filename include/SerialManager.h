#ifndef SERIALMGR_H
#define SERIALMGR_H

#include "Message.h"
#include "ConfigSuite.h"

enum dState {done, reading, seek};

class SerialMgr{
  public:

  void ReceiveMsg();
  void SendMsg(uint8_t code);
  void SendMsg(uint8_t code, uint16_t opt);
  void SendMsg(uint8_t code, uint16_t opt, uint8_t* data);

  private:

  uint8_t bx = 0;
  uint8_t rxlen = 0;
  uint8_t recvbytes[256];
  uint8_t recvdata[256];
  uint8_t inb;
  dState state = seek;

  FC_cfg txcfg;

  void data_seek();
  void data_read();
  void data_done();
};

#endif // SERIALMGR_H
