#include "SigmaGPIO.h"
#include "SigmaIO.h"

SigmaGPIO::SigmaGPIO()
{
}

SigmaGPIO::~SigmaGPIO()
{
}

void SigmaGPIO::PinMode(uint pin, byte mode)
{
    pinMode(pin, mode);
}

IOError SigmaGPIO::DigitalWrite(uint pin, byte value)
{
    if (ESP_OK == gpio_set_level((gpio_num_t)pin, value))
    {
        return SIGMAIO_SUCCESS;
    }
    return SIGMAIO_ERROR_BAD_VALUE;
}

byte SigmaGPIO::DigitalRead(uint pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

bool SigmaGPIO::CanBePWM(uint pin)
{
    return GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

IOError SigmaGPIO::AnalogWrite(uint pin, uint value)
{
    analogWrite(pin, value);
    return SIGMAIO_SUCCESS;
}

int SigmaGPIO::AnalogRead(uint pin)
{
    return analogRead(pin);
}

bool SigmaGPIO::RegisterPwmPin(uint pin, uint frequency, byte resolution, uint minValue, uint maxValue)
{
    ChannelDefinition channelDefinition = {0, frequency, resolution, minValue, maxValue};
    byte minCh = 0;
    bool chFound = false;
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin already registered.
            return false;
        }
        if (!chFound)
        {
            if (channel.second.number == minCh)
            {
                minCh++;
            }
            else if (channel.second.number > minCh)
            {
                chFound = true;
            }
        }
    }

    // pin not registered yet.
    channelDefinition.number = minCh;
    pwmChannels.insert(std::pair<byte, ChannelDefinition>(pin, channelDefinition));
    ledcSetup(channelDefinition.number, channelDefinition.frequency, channelDefinition.resolution);
    ledcAttachPin(pin, channelDefinition.number);
    return true;
}

bool SigmaGPIO::UnRegisterPwmPin(uint pin)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found
            pwmChannels.erase(channel.first);
            return true;
        }
    }
    return false;
}

bool SigmaGPIO::SetPwm(uint pin, uint value)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            ledcWrite(channel.second.number, NormalizePwmValue(value, channel.second.resolution, channel.second.minValue, channel.second.maxValue));
            return true;
        }
    }
    return false;
}

