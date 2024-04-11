#pragma once

enum ChannelType {
    AXIS,
    BUTTON
};

struct ChannelParameters {
    ChannelType type;
    float output_min = 0;
    float output_max = 1;
    float input_min = 1000;
    float input_max = 2000;
    float input_center = (input_max - input_min)/2;
    float deadband = 0;
};

class Channel {
private:
    ChannelParameters params;
    float input;
    float output;
public:
    Channel();
    Channel(ChannelParameters p);
    void setInput(float i);
    float getOutput();
};