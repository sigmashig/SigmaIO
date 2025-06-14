#pragma once
#include "SigmaIODriver.h"
#include "SigmaIOTypes.h"
#include <map>

class SigmaGPIO : public SigmaIODriver
{
public:
    SigmaGPIO();
    ~SigmaGPIO();
    void PinMode(uint pin, byte mode);
    IOError DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin);
    bool CanBePWM(uint pin);
    IOError AnalogWrite(uint pin, uint value);
    int AnalogRead(uint pin);
    bool RegisterPwmPin(uint pin, uint frequency = 5000);
    bool UnRegisterPwmPin(uint pin);

    bool SetPwm(uint pin, uint value);
    String GetPinDriverName() { return "SigmaGPIO"; }
    uint GetNumberOfPins() { return GPIO_PIN_COUNT; }
    void AfterRegistration(PinDriverDefinition pdd) {};

private:
    typedef struct
    {
        byte number;
        uint frequency;
        byte resolution = 7;
        uint minValue = 0;
        uint maxValue = 0x7F;
    } ChannelDefinition;
    std::map<byte, ChannelDefinition> pwmChannels;
};