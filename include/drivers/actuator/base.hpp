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
    virtual void setThrottle(float throttle) = 0;
    virtual void setPitch(float pitch) = 0;
    virtual void setRoll(float roll) = 0;
    virtual void setYaw(float yaw) = 0;
    virtual void setArm(bool arm) = 0;
    virtual void setMode(bool mode) = 0;
    virtual void setConfig(const void* config) = 0;
    virtual void update() = 0;
private:
    ActuatorType type;
};