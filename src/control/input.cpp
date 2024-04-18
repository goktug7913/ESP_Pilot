#include "control/input.hpp"
#include "drivers/receiver/pwm.hpp"

static const char* TAG = "InputSystem";

InputSystem::InputSystem(std::vector<Channel> c) {
    num_channels = c.size();
    channels = c;
    init();
}

/**
 * Initialize the driver
*/
void InputSystem::init() {
    rx_driver = new PWMReceiver();
}