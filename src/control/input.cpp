#include "control/input.hpp"

InputSystem::InputSystem() {
    params = InputParameters();
}

InputSystem::InputSystem(InputParameters p) {
    params = p;
}

void InputSystem::setParameters(InputParameters p) {
    params = p;
}

InputParameters InputSystem::getParameters() {
    return params;
}

float InputSystem::map(float input) {
    return map(input, params.output_min, params.output_max);
}

float InputSystem::map(float input, float output_min, float output_max) {
    float output = (input - params.input_min) * (output_max - output_min) / (params.input_max - params.input_min) + output_min;
    if (params.deadband > 0) {
        if (output < params.output_center + params.deadband && output > params.output_center - params.deadband) {
            output = params.output_center;
        }
    }
    return output;
}