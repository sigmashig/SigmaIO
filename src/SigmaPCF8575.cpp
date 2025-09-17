#include "SigmaPCF8575.h"
#include "SigmaIO.h"
#include <Wire.h>

SigmaPCF8575IO::SigmaPCF8575IO(byte address, uint isrPin, TwoWire *pWire, uint sda, uint scl)
{
    this->isrPin = isrPin;
    if (pWire == nullptr)
    {
        Wire.begin(sda, scl);
        pWire = &Wire;
    }
    pcf8575 = new PCF8575(pWire, address, sda, scl);
    pcf8575->begin();
    if (isrPin != 0)
    {
        SigmaIO::PinMode(isrPin, INPUT_PULLUP);
        //        SigmaIO::AttachInterrupt(isrPin, isrPin, 100, FALLING);
    }
}

SigmaPCF8575IO::~SigmaPCF8575IO()
{
    delete pcf8575;
}

void SigmaPCF8575IO::PinMode(uint pin, byte mode)
{
    byte m = mode == OUTPUT ? OUTPUT : INPUT; // Pull-up/down is not supported
    pcf8575->pinMode(pin, m);
    int oldMode = SigmaIO::GetPinMode(pin);
    if ((oldMode & INPUT) && m == OUTPUT)
    {
        SigmaIO::DetachInterrupt(this->isrPin, pin);
    }
    if ((oldMode & OUTPUT) && m == INPUT)
    {
        SigmaIO::AttachInterrupt(this->isrPin, pin, 100, FALLING);
    }
}

IOError SigmaPCF8575IO::DigitalWrite(uint pin, byte value)
{
    // Serial.printf("DigitalWrite: pin: %d, value: %d\n", pin, value);
    if (pcf8575->digitalWrite(pin, value))
    {
        return SIGMAIO_SUCCESS;
    }
    return SIGMAIO_ERROR_BAD_VALUE;
}

byte SigmaPCF8575IO::DigitalRead(uint pin)
{
    return pcf8575->digitalRead(pin);
}
