#pragma once
#include <Arduino.h>
#include "SigmaIOTypes.h"
#include <PCF8575.h>
#include "SigmaIODriver.h"

class SigmaPCF8575IO : public SigmaIODriver
{
public:
    SigmaPCF8575IO(byte address, uint isrPin = 0, TwoWire *pWire = nullptr, uint sda = 0, uint scl = 0);

    ~SigmaPCF8575IO();
    String GetPinDriverName() { return "SigmaPCF8575IO"; }
    void PinMode(uint pin, byte mode);
    IOError DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin);
    bool CanBePWM(uint pin) { return false; };
    IOError AnalogWrite(uint pin, uint value) { return SIGMAIO_ERROR_NOT_SUPPORTED; };
    int AnalogRead(uint pin) { return -1; };
    bool RegisterPwmPin(uint pin, uint frequency) { return false; }
    bool UnRegisterPwmPin(uint pin) { return false; };

    bool SetPwmRaw(uint pin, uint value) { return false; };
    bool SetPwmPercent(uint pin, uint value) { return false; };
    uint GetNumberOfPins() { return 16; }
    void AfterRegistration(PinDriverDefinition pdd) {};
    bool SetPwmUSec(uint pin, uint value) { return false; };
    uint GetPwmFrequency(uint pin){ return 0; };
    uint GetMinPwmFrequency(uint pin){ return 0; };
    uint GetMaxPwmFrequency(uint pin){ return 0; };
    uint GetPwmResolution(uint pin){ return 0; };
    bool IsPinPWM(uint pin){ return false; };
    uint GetMaxPwmValue(uint pin){ return 0; };
    uint GetMinPwmValue(uint pin){ return 0; };

private:
    PCF8575 *pcf8575;
    uint isrPin;
};