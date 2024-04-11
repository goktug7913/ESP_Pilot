#include "control/input.hpp"
#include "drivers/receiver/pwm.hpp"

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