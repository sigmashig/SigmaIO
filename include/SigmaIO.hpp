#pragma once
#include <Arduino.h>
#include "SigmaAbstractPinDriver.hpp"
// #include <vector>
#include <map>

typedef enum
{
    SIGMAIO_SUCCESS = 0,
    SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED,
    SIGMAIO_ERROR_PIN_NOT_REGISTERED,
    SIGMAIO_ERROR_PIN_DRIVER_ALREADY_REGISTERED,
    SIGMAIO_ERROR_BAD_PIN_RANGE,
    SIGMAIO_ERROR_BAD_PIN_DRIVER,
    SIGMAIO_ERROR_PIN_NOT_PWM
} IOError;

class SigmaIO
{
public:
    SigmaIO();
    ~SigmaIO();
    IOError PinMode(uint pin, byte mode);
    void DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin);
    IOError RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd);
    IOError UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver);
    IOError RegisterPwmPin(uint pin, uint frequency, byte resolution, uint minValue = 0, uint maxValue = 0xFFFF);
    IOError SetPwm(uint pin, uint value);

private:
    typedef struct
    {
        uint beg;
        uint end;
        SigmaAbstractPinDriver *pinDriver;
    } PinDriverDefinition;
    std::map<uint, PinDriverDefinition> pinRangeDriverSet;
    std::map<uint, PinDriverDefinition> pinDriverSet;
    PinDriverDefinition getPinDriver(uint pin);
};

extern SigmaIO *sigmaIO;