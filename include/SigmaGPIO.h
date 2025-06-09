#pragma once
#include "SigmaIOTypes.h"
#include "SigmaAbstractPinDriver.h"
#include <map>

class SigmaGPIO : public SigmaAbstractPinDriver
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
    bool RegisterPwmPin(uint pin, uint frequency = 5000, byte resolution = 7, uint minValue = 0, uint maxValue = 0xFFFF);
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
        byte resolution;
        uint minValue;
        uint maxValue;
    } ChannelDefinition;
    std::map<byte, ChannelDefinition> pwmChannels;
};