#include "SigmaPCF8575.h"
#include "SigmaIO.h"
#include <Wire.h>

SigmaPCF8575IO::SigmaPCF8575IO(byte address, TwoWire *pWire, uint sda, uint scl)
{
    if (pWire == nullptr)
    {
        Wire.begin();
        pcf8575 = new PCF8575(address);
    }
    else
    {
        pcf8575 = new PCF8575(pWire, address, sda, scl);
    }
    pcf8575->begin();
    // init();
}

SigmaPCF8575IO::~SigmaPCF8575IO()
{
    delete pcf8575;
}

void SigmaPCF8575IO::PinMode(uint pin, byte mode)
{
    byte m = mode == OUTPUT ? OUTPUT : INPUT; // Pull-up/down is not supported
    pcf8575->pinMode(pin, m);
}

IOError SigmaPCF8575IO::DigitalWrite(uint pin, byte value)
{
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
