#include "control/channel.hpp"

Channel::Channel() {
    params = ChannelParameters();
}

Channel::Channel(ChannelParameters p) {
    params = p;
}

void Channel::setInput(float i) {
    if (i < params.input_min) {
        input = params.input_min;
    } else if (i > params.input_max) {
        input = params.input_max;
    } else {
        if (params.deadband > 0) {
            if (i < params.input_center + params.deadband && i > params.input_center - params.deadband) {
                input = params.input_center;
            } else {
                input = i;
            }
        } else {
            input = i;
        }
    }

    switch (params.type) {
    case ChannelType::AXIS:
        output = (input - params.input_min) * (params.output_max - params.output_min) / (params.input_max - params.input_min) + params.output_min;
        break;
    case ChannelType::BUTTON:
        // We assume that the button is pressed if the input is greater than half of the input range
        output = input > params.input_center ? 1.0f : 0.0f;
        break;
    default:
        // This should never happen
        input = params.input_min;
        break;
    }
}

float Channel::getOutput() {
    return output;
}