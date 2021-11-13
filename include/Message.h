#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>
#pragma once

// A Serial frame is structured like this:
// Length is size of
// start indicates the starting of header
// data_start indicates that bytes until data_end is binary data
// end indicates the end of the whole packet

struct msg_begin{
  uint8_t  start = 0;         //Message Start Flag (0xB1)
  uint32_t length = 0;        //Message Length
  uint8_t  cmd = 0;           //Command
  uint16_t opt = 0;           //Options
  uint8_t  data_start = 0;    //Data Start Flag (0xD2)
};

// Binary data will be here, length could vary

struct msg_end{
  uint8_t  data_end = 0;    //Data End Flag (0xD1)
  uint8_t  end = 0;         //Message End Flag (0xB2)
};

#endif // MESSAGE_H