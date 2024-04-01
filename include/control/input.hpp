#pragma once
struct InputParameters {
    float output_min = 0;
    float output_max = 1;
    float input_min = 1000;
    float input_max = 2000;
    float deadband = 0;
    float input_center = input_max - input_min / 2;
    float output_center = output_max - output_min / 2;
};

class InputSystem {
private:
    InputParameters params;
public:
    InputSystem();
    InputSystem(InputParameters p);

    void setParameters(InputParameters p);
    InputParameters getParameters();
    
    float map(float input);
    float map(float input, float output_min, float output_max);

};