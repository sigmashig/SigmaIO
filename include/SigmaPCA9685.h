#pragma once
#include <Arduino.h>
#include "SigmaIOTypes.h"
#include <PCA9685.h>
#include "SigmaIODriver.h"

class SigmaPCA9685IO : public SigmaIODriver
{
public:
    SigmaPCA9685IO(byte address, uint frequency = 0, TwoWire *pWire = nullptr);

    ~SigmaPCA9685IO();
    String GetPinDriverName() { return "SigmaPCA9685IO"; }
    /**
     * @brief Set the pin mode. IGNORED! All pins are OUTPUT PWM
     * @param pin - the pin to set the mode for
     * @param mode - the mode to set: OUT
     */
    void PinMode(uint pin, byte mode) {};
    IOError DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin) { return 255; };
    bool CanBePWM(uint pin) { return true; };
    IOError AnalogWrite(uint pin, uint value) { return SIGMAIO_ERROR_NOT_SUPPORTED; };
    int AnalogRead(uint pin) { return -1; };
    /**
     * @brief Register a pin as PWM is ignored - all pins are PWM
     * @param pin - the pin to register
     * @param frequency - IGNORED! Should be set for driver
     * @param resolution - IGNORED! Fixed! resolution = 12
     * @param minValue - IGNORED! Fixed! minValue = 0
     * @param maxValue - IGNORED! Fixed! maxValue = 0x0FFF
     */
    bool RegisterPwmPin(uint pin, uint frequency = 0) { return true; };
    bool UnRegisterPwmPin(uint pin) { return true; };

    bool SetPwm(uint pin, uint value);
    uint GetNumberOfPins() { return 16; }
    void AfterRegistration(PinDriverDefinition pdd);

private:
    PCA9685 *pca9685;
};