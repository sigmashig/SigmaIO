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

    bool SetPwmRaw(uint pin, uint value);
    String GetPinDriverName() { return "SigmaGPIO"; }
    uint GetNumberOfPins() { return GPIO_PIN_COUNT; }
    void AfterRegistration(PinDriverDefinition pdd) {};
    //TODO: Implement these methods
    bool SetPwmUSec(uint pin, uint value){ return false; };
    uint GetPwmFrequency(uint pin){ return 0; };
    uint GetMinPwmFrequency(uint pin){ return 0; };
    uint GetMaxPwmFrequency(uint pin){ return 0; };
    uint GetPwmResolution(uint pin){ return 0; };
    bool IsPinPWM(uint pin){ return false; };
    uint GetMaxPwmValue(uint pin){ return 0x7F; };
    uint GetMinPwmValue(uint pin){ return 0; };
    bool SetPwmPercent(uint pin, uint value);

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