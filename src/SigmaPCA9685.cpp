#include "SigmaPCA9685.h"
#include "SigmaIO.h"

SigmaPCA9685IO::SigmaPCA9685IO(byte address, uint frequency, TwoWire *pWire)
{
    if (pWire == nullptr)
    {
        pca9685 = new PCA9685(address);
    }
    else
    {
        pca9685 = new PCA9685(address, pWire);
    }

    // Serial.println("PCA9685 created");
    if (frequency > 0)
    {
        pca9685->setFrequency(frequency);
    }
    pca9685->begin();
    // Serial.println("PCA9685 initialized");
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

bool SigmaPCA9685IO::SetPwm(uint pin, uint value)
{
    uint normalizedValue = NormalizePwmValue(value, 0, 0x0FFF);
    // Serial.printf("SetPwm: pin: %d, value: %d, normalizedValue: %d\n", pin, value, normalizedValue);
    if (pca9685->setPWM(pin, normalizedValue) == 0)
    {
        return true;
    }
    return false;
}

void SigmaPCA9685IO::AfterRegistration(PinDriverDefinition pdd)
{
    // Serial.println("PCA9685 after registration");
    for (int i = 0; i < 16; i++)
    {
        // SigmaIO::PinMode(i + pdd.beg, OUTPUT);
        SigmaIO::RegisterPwmPin(i + pdd.beg);
    }
}
