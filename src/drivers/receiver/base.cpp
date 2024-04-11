#include "drivers/receiver/base.hpp"

void ReceiverBase::setChannel(uint8_t channel, uint16_t value) {
    // Set the value of a specific channel
    if (channel < numChannels) {
        return;
    }
}