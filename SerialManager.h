#ifndef SERIALMGR_H
#define SERIALMGR_H

#include "Message.h"

class SerialMgr{
  public:

  void ReceiveCmd();

  void SignalEvent(uint8_t code);

  private:


};

#endif // SERIALMGR_H