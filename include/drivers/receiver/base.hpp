#pragma once
#include <stdint.h>

class ReceiverBase {
public:
    virtual void init() = 0;
    virtual void read() = 0;
    virtual void setChannel(uint8_t channel, uint16_t value) = 0;
    virtual uint16_t getChannel(uint8_t channel) = 0;
    virtual ~ReceiverBase() = default;
private:
    uint16_t channels[16];
};