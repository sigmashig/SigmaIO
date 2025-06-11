#include "SigmaAbstractPinDriver.h"

uint SigmaAbstractPinDriver::NormalizePwmValue(uint value, uint minValue, uint maxValue)
{
    uint res;
    if (value == 0)
    {
        res = 0;
    }
    else
    {
        uint newValue = min<uint>(value, 100);
        res = map(newValue, 0, 100, minValue, maxValue);
    }
    //Serial.printf("NormalizePwmValue: value: %d, minValue: %d, maxValue: %d, res: %d\n", value, minValue, maxValue, res);
    return res;
}