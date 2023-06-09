#include "SigmaAbstractPinDriver.hpp"

uint SigmaAbstractPinDriver::NormalizePwmValue(uint value, byte resolution, uint minValue, uint maxValue)
{
    uint res;
    uint maxVal = min<uint>(((((uint)1) << resolution) - 1), maxValue);
    res = map((value > 99 ? 99 : value), 0, 99, minValue, maxVal);
    return res;
}