#include "drivers/receiver/base.hpp"

class PWMReceiver : public ReceiverBase {
public:
    PWMReceiver();
    ~PWMReceiver() override = default;
    void init() override;
    void read() override;

protected:
    void setChannel(uint8_t channel, uint16_t value);
};