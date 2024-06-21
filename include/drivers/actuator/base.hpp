#pragma once

enum class ActuatorType {
    PWM,
    DSHOT,
    GPIO,
    I2C,
    SPI,
};

class ActuatorBase {
public:
    ActuatorBase(ActuatorType type) : type(type) {}
    virtual void write(float value) = 0;
private:
    ActuatorType type;
    
};