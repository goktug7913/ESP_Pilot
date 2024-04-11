#pragma once
#include "drivers/receiver/base.hpp"
#include "control/channel.hpp"
#include <vector>

struct InputLayout {
    int num_channels;
    std::vector<Channel> channels;
};

class InputSystem {
private:
    ReceiverBase* rx_driver;
    int num_channels;
    void init();
protected:
    std::vector<Channel> channels;
public:
    InputSystem(std::vector<Channel> c);
};