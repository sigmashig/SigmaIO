#include "SigmaIO.hpp"

SigmaIO::SigmaIO()
{
}

SigmaIO::~SigmaIO()
{
}

IOError SigmaIO::PinMode(uint pin, byte mode)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    IOError res = SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    if (pdd.pinDriver == nullptr)
    {
        // pin is not initialized yet
        for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
        {
            PinDriverDefinition pdd = it->second;

            if (it->first <= pin)
            {
                if (pin <= pdd.end)
                { // range found
                    std::pair<int, PinDriverDefinition> newPair = {pin, pdd};
                    pinDriverSet.insert(newPair);
                    res = SIGMAIO_SUCCESS;
                    break;
                }
            }
        }
    }
    if (res == SIGMAIO_SUCCESS)
    {
        pdd.pinDriver->PinMode(pin - pdd.beg, mode);
    }
    return res;
}

void SigmaIO::DigitalWrite(uint pin, byte value)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        pdd.pinDriver->DigitalWrite(pin - pdd.beg, value);
    }
}

byte SigmaIO::DigitalRead(uint pin)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        return pdd.pinDriver->digitalRead(pin - pdd.beg);
    }
    return 0xFF;
}

IOError SigmaIO::RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd)
{
    // PinDriverRange range = {pinBegin, pinEnd};
    if (pinBegin > pinEnd)
    {
        return SIGMAIO_ERROR_BAD_PIN_RANGE;
    }
    if (pinDriver == NULL)
    {
        return SIGMAIO_ERROR_BAD_PIN_DRIVER;
    }
    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        PinDriverDefinition pdd = it->second;
        if (pdd.pinDriver == pinDriver)
        {
            if (pdd.beg == pinBegin && pdd.end == pinEnd)
            {
                return SIGMAIO_SUCCESS;
            }
            else
            {
                return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
            }
        }
        if (it->first > pinBegin)
        { // potentially found a hole. check if it is not overlapping with end
            if (it->first <= pinEnd)
            {
                return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
            }
            else
            {
                std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, pinDriver}};
                pinRangeDriverSet.insert(newPair);
                return SIGMAIO_SUCCESS;
            }
        }
    }
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver)
{
    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        PinDriverDefinition pdd = it->second;
        if (pdd.pinDriver == pinDriver)
        {
            for (auto it1 = pinDriverSet.begin(); it1 != pinDriverSet.end(); it1++)
            {
                if (it1->second.pinDriver == pinDriver)
                {
                    pinDriverSet.erase(it1);
                }
            }
            pinRangeDriverSet.erase(it);
            return SIGMAIO_SUCCESS;
        }
    }

    return SIGMAIO_ERROR_BAD_PIN_DRIVER;
}

IOError SigmaIO::RegisterPwmPin(uint pin, uint frequency, byte resolution, uint minValue, uint maxValue)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        if (pdd.pinDriver->CanBePWM(pin - pdd.beg))
        {
            if (pdd.pinDriver->RegisterPwmPin(pin - pdd.beg, frequency, resolution, minValue, maxValue))
            {
                return SIGMAIO_SUCCESS;
            }
            else
            {
                return SIGMAIO_ERROR_PIN_NOT_PWM;
            }
        }
        else
        {
            return SIGMAIO_ERROR_PIN_NOT_PWM;
        }
    }
    else
    {
        return SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    }
}

IOError SigmaIO::SetPwm(uint pin, uint value)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        if (pdd.pinDriver->SetPwm(pin - pdd.beg, value))
        {
            return SIGMAIO_SUCCESS;
        }
        else
        {
            return SIGMAIO_ERROR_PIN_NOT_PWM;
        }
    }
    else
    {
        return SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    }
}
SigmaIO::PinDriverDefinition SigmaIO::getPinDriver(uint pin)
{
    if (pinDriverSet.size() != 0)
    {
        auto it = pinDriverSet.find(pin);
        if (it != pinDriverSet.end())
        {
            return it->second;
        }
    }
    return {0, 0, nullptr};
}

//-----------------------------------------------------
SigmaIO *sigmaIO;
