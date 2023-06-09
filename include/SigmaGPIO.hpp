#pragma once
#include "SigmaAbstractPinDriver.hpp"
#include <map>

class SigmaGPIO : public SigmaAbstractPinDriver
{
public:
    SigmaGPIO();
    ~SigmaGPIO();
    void PinMode(byte pin, byte mode);
    void DigitalWrite(byte pin, byte value);
    byte DigitalRead(byte pin);
    bool CanBePWM(byte pin);
    void AnalogWrite(byte pin, uint value);
    int AnalogRead(byte pin);
    bool RegisterPwmPin(byte pin, uint frequency = 5000, byte resolution = 7, uint minValue = 0, uint maxValue = 0xFFFF);
    bool UnRegisterPwmPin(byte pin);

    bool SetPwm(byte pin, uint value);
    String GetPinDriverName() { return "SigmaGPIO"; }
    bool Begin() { return true; };

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