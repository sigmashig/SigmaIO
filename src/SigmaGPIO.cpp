#include "SigmaGPIO.hpp"
#include "SigmaIO.hpp"

SigmaGPIO::SigmaGPIO()
{
}

SigmaGPIO::~SigmaGPIO()
{
}

void SigmaGPIO::PinMode(byte pin, byte mode)
{
    pinMode(pin, mode);
}

void SigmaGPIO::DigitalWrite(byte pin, byte value)
{
    gpio_set_level((gpio_num_t)pin, value);
}

byte SigmaGPIO::DigitalRead(byte pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

bool SigmaGPIO::CanBePWM(byte pin)
{
    return GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

void SigmaGPIO::AnalogWrite(byte pin, uint value)
{
    analogWrite(pin, value);
}

int SigmaGPIO::AnalogRead(byte pin)
{
    return analogRead(pin);
}

bool SigmaGPIO::RegisterPwmPin(byte pin, uint frequency, byte resolution, uint minValue, uint maxValue)
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

bool SigmaGPIO::UnRegisterPwmPin(byte pin)
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

bool SigmaGPIO::SetPwm(byte pin, uint value)
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

