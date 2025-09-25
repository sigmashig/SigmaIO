#include "SigmaIODriver.h"

uint SigmaIODriver::NormalizePwmValue(uint value, uint minValue, uint maxValue)
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
    return res;
}