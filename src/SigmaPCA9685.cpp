#include "SigmaPCA9685.h"
#include "SigmaIO.h"

SigmaPCA9685IO::SigmaPCA9685IO(byte address, uint frequency, TwoWire *pWire, uint sda, uint scl)
{
    if (pWire == nullptr)
    {
        bool res = Wire.begin(sda, scl);
        pWire = &Wire;
    }
    pca9685 = new PCA9685(address, pWire);

    if (frequency > 0)
    {
        pca9685->setFrequency(frequency);
    }
    pca9685->begin();
}

SigmaPCA9685IO::~SigmaPCA9685IO()
{
    delete pca9685;
}

IOError SigmaPCA9685IO::DigitalWrite(uint pin, byte value)
{
    if (pca9685->write1(pin, value) == 0)
    {
        return SIGMAIO_SUCCESS;
    }
    return SIGMAIO_ERROR_BAD_VALUE;
}

bool SigmaPCA9685IO::SetPwmPercent(uint pin, uint value)
{
    uint normalizedValue = NormalizePwmValue(value, 0, 0x0FFF);
    if (pca9685->setPWM(pin, normalizedValue) == 0)
    {
        return true;
    }
    return false;
}

bool SigmaPCA9685IO::SetPwmRaw(uint pin, uint value)
{
    if (pca9685->setPWM(pin, value) == 0)
    {
        return true;
    }
    return false;
}

void SigmaPCA9685IO::AfterRegistration(PinDriverDefinition pdd)
{
    for (int i = 0; i < 16; i++)
    {
        // SigmaIO::PinMode(i + pdd.beg, OUTPUT);
        SigmaIO::RegisterPwmPin(i + pdd.beg);
    }
}

bool SigmaPCA9685IO::SetPwmUSec(uint pin, uint value)
{
    uint maxWidth = (1.0f * 1e6f) / pca9685->getFrequency();
    uint percentValue;
    if (value > maxWidth)
    {
        percentValue = 100;
        // return false;
    }
    else
    {
        percentValue = value * 100 / maxWidth;
    }
    return SetPwmPercent(pin, percentValue);
}

uint SigmaPCA9685IO::GetPwmFrequency(uint pin)
{
    return pca9685->getFrequency();
}
