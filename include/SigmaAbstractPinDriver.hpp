#pragma once
#include <Arduino.h>

class SigmaAbstractPinDriver
{
public:
    // virtual String GetPinDriverName() = 0;
    virtual void PinMode(byte pin, byte mode) = 0;
    virtual void DigitalWrite(byte pin, byte value) = 0;
    virtual byte DigitalRead(byte pin) = 0;
    virtual bool CanBePWM(byte pin) = 0;
    virtual void AnalogWrite(byte pin, uint value) = 0;
    virtual int AnalogRead(byte pin) = 0;
    virtual bool RegisterPwmPin(byte pin, uint frequency, byte resolution, uint minValue = 0, uint maxValue = 0xFFFF) = 0;
    virtual bool UnRegisterPwmPin(byte pin) = 0;
    virtual bool SetPwm(byte pin, uint value) = 0;

protected:
    static uint NormalizePwmValue(uint value, byte resolution, uint minValue = 0, uint maxValue = 0xFFFF);
};