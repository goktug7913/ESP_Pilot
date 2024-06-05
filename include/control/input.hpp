#pragma once
#include "drivers/receiver/base.hpp"
#include "control/channel.hpp"
#include <vector>

struct InputLayout {
    RxType rx_type;
    int num_channels;
    std::vector<Channel> channels;
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
    void read();
};