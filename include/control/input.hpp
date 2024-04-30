#pragma once
#include "drivers/receiver/base.hpp"
#include "control/channel.hpp"
#include <vector>

struct InputLayout {
    int num_channels;
    std::vector<Channel> channels;
    RxType rx_type;
};

class InputSystem {
private:
    ReceiverBase* rx_driver;
    void init();
protected:
    InputLayout layout;
public:
    InputSystem();
    InputSystem(InputLayout l);
};