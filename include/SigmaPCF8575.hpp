#pragma once
#include <Arduino.h>
#include "SigmaAbstractPinDriver.hpp"
#include <PCF8575.h>

class SigmaPCF8575IO : public SigmaAbstractPinDriver
{
public:
    SigmaPCF8575IO(byte address);
    SigmaPCF8575IO(TwoWire *pWire, byte address, uint sda = 0, uint scl = 0);

    ~SigmaPCF8575IO();
    String GetPinDriverName() { return "SigmaPCF8575IO"; }
    bool Begin();
    void PinMode(byte pin, byte mode);
    void DigitalWrite(byte pin, byte value);
    byte DigitalRead(byte pin);
    bool CanBePWM(byte pin) { return false; };
    void AnalogWrite(byte pin, uint value){};
    int AnalogRead(byte pin) { return -1; };
    bool RegisterPwmPin(byte pin, uint frequency, byte resolution, uint minValue = 0, uint maxValue = 0xFFFF) { return false; }
    bool UnRegisterPwmPin(byte pin) { return false; };

    bool SetPwm(byte pin, uint value) { return false; };

private:
    PCF8575 *pcf8575;
};