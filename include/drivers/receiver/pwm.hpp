#pragma once
#include "drivers/receiver/base.hpp"

class PWMReceiver : public virtual ReceiverBase {
public:
    PWMReceiver();
    ~PWMReceiver() override = default;
    void init() override;
    void read() override;
    float stub;
private:
};