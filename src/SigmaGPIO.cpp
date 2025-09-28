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
    // return GPIO_IS_VALID_OUTPUT_GPIO(pin);
    return (pin == 2 || pin == 4 || pin == 12 || pin == 13 || pin == 14 || pin == 15 || pin == 16 || pin == 17 || pin == 18 || pin == 19 || pin == 21 || pin == 22 || pin == 23 || pin == 25 || pin == 26 || pin == 27 || pin == 32 || pin == 33);
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

bool SigmaGPIO::RegisterPwmPin(uint pin, uint frequency)
{
    ChannelDefinition channelDefinition = {
        .number = 0,
        .frequency = frequency};
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

bool SigmaGPIO::SetPwmPercent(uint pin, uint value)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            if (value > 100)
            {
                value = 100;
            }
            uint nValue = NormalizePwmValue(value, channel.second.minValue, channel.second.maxValue);
            // Serial.printf("SetPwm: pin: %d, value: %d, nValue: %d\n", pin, value, nValue);
            ledcWrite(channel.second.number, nValue);
            return true;
        }
    }
    return false;
}

uint SigmaGPIO::GetPwmFrequency(uint pin)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            return channel.second.frequency;
        }
    }
    return 0;
}

bool SigmaGPIO::IsPinPWM(uint pin)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            return true;
        }
    }
    return false;
}

uint SigmaGPIO::GetPwmResolution(uint pin)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            return channel.second.resolution;
        }
    }
    return 0;
}

bool SigmaGPIO::SetPwmUSec(uint pin, uint value)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            ledcWrite(channel.second.number, value);
            ledcWriteTone(channel.second.number, value);
            return true;
        }
    }
    return false;
}

bool SigmaGPIO::SetPwmRaw(uint pin, uint value)
{
    for (auto &channel : pwmChannels)
    {
        if (channel.first == pin)
        { // pin found.
            ledcWrite(channel.second.number, value);
            return true;
        }
    }
    return false;
}
