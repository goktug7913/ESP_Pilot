#pragma once
#include <stdint.h>
#include <vector>

/*  - note -
    TODO: I need to analyze this:
    If we determine the required driver class at runtime,
    the compiler will have to include all the driver classes
    in the final binary. This is not a problem for small
    projects, but it can be a problem for larger projects.

    If we determine the required driver class at compile time,
    We lose the flexibility of changing the driver at runtime.
    To use a different driver, we have to recompile the binary.

    Maybe we can use a factory pattern to create the driver.
    Another option is to dynamicly load the driver at runtime.
    But that would require a more complex architecture,
    which might be an overkill for this project.
*/

enum RxType {
    //PPM,
    PWM,
    //SBUS,  // Futaba S.BUS
    //SUMD,  // Graupner SUMD 
    //IBUS,  // FlySky IBUS
    // ... add more as needed
    // Only PWM is implemented right now, so others are commented out
};

// This is the base class for all types of receivers.
// It must not be used directly and always be derived.
class ReceiverBase {
public:
    virtual ~ReceiverBase() = default; 
    virtual void init() = 0;
    virtual bool isConnected() { return false; };
    virtual uint16_t getChannel(uint8_t channel) = 0;
protected:
    RxType type;   // Receiver protocol type
    uint8_t numChannels; // Number of supported channels
    void setChannel(uint8_t channel, uint16_t value);
};