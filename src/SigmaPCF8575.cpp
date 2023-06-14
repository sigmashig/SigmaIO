#include "SigmaPCF8575.hpp"
#include "SigmaIO.hpp"

SigmaPCF8575IO::SigmaPCF8575IO(byte address)
{
    pcf8575 = new PCF8575(address);
    init();
}

SigmaPCF8575IO::SigmaPCF8575IO(TwoWire *pWire, byte address, uint sda, uint scl)
{
    pcf8575 = new PCF8575(pWire, address, sda, scl);
    init();
}

SigmaPCF8575IO::~SigmaPCF8575IO()
{
    delete pcf8575;
}

void SigmaPCF8575IO::PinMode(byte pin, byte mode)
{
    byte m = mode == OUTPUT ? OUTPUT : INPUT;
    pcf8575->pinMode(pin, m);
}

void SigmaPCF8575IO::DigitalWrite(byte pin, byte value)
{
    pcf8575->digitalWrite(pin, value);
}

byte SigmaPCF8575IO::DigitalRead(byte pin)
{
    return pcf8575->digitalRead(pin);
}

bool SigmaPCF8575IO::Begin()
{
    pcf8575->begin();
    return true;
}
