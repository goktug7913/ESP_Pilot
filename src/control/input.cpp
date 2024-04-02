#include "control/input.hpp"
#include "drivers/receiver/pwm.hpp"

InputSystem::InputSystem() {
    params = InputParameters();
    init();
}

InputSystem::InputSystem(InputParameters p) {
    params = p;
    init();
}

/**
 * Set the current parameters
*/
void InputSystem::setParameters(InputParameters p) {
    params = p;
}

/**
 * Get the current parameters
*/
InputParameters InputSystem::getParameters() {
    return params;
}

/**
 * Map the input value to the output range
*/
float InputSystem::map(float input) {
    return map(input, params.output_min, params.output_max);
}

/**
 * Map the input value to the output range
*/
float InputSystem::map(float input, float output_min, float output_max) {
    float output = (input - params.input_min) * (output_max - output_min) / (params.input_max - params.input_min) + output_min;

    if (params.deadband > 0) {
        if (output < params.output_center + params.deadband && output > params.output_center - params.deadband) {
            output = params.output_center;
        }
    }

    return output;
}

/**
 * Initialize the driver
*/
void InputSystem::init() {
    rx_driver = new PWMReceiver();
}