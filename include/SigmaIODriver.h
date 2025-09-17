#pragma once
#include <Arduino.h>
#include "SigmaIOTypes.h"

class SigmaIODriver
{
public:
    /**
     * @brief Get the Pin Driver Name object. This method is not used inside IO system. You can use it for debug purposes
     * @return String
     */
    virtual String GetPinDriverName() = 0;
    /**
     * @brief Set pin mode. This method is used for registration pin in the IO system. Should be call before read/write/attach operatons
     * No default mode is set. You should set it before use
     * @param pin - number of pin
     * @param mode - pin mode. See Arduino pinMode() function for details. Attention! Not all modes are supported by all drivers.
     *         The INPUT and OUTPUT should be supported by all drivers. Support of INPUT_PULLUP etc is not guarantied.
     *
     */
    virtual void PinMode(uint pin, byte mode) = 0;
    /**
     * @brief Write digital value to the pin. This method is used for registered pins only.
     */
    virtual IOError DigitalWrite(uint pin, byte value) = 0;
    /**
     * @brief Read digital value from the pin. This method is used for registered pins only.
     */
    virtual byte DigitalRead(uint pin) = 0;
    /**
     * @brief Check if pin can be used as PWM. This method is used for registered pins only.
     * @return true when the PWM is supported
     */
    virtual bool CanBePWM(uint pin) = 0;
    /**
     * @brief Check if pin is PWM. This method is used for registered pins only.
     * @return true when the pin is PWM
     */
    virtual bool IsPinPWM(uint pin) = 0;
    /**
     * @brief Write analog value to the pin. This method is used for registered pins only.
     *        Attention! Not all drivers supports of analog write. You should check it before use
     */

    virtual IOError AnalogWrite(uint pin, uint value) = 0;
    /**
     * @brief Read analog value from the pin. This method is used for registered pins only.
     *        Attention! Not all drivers supports of analog read. You should check it before use
     */
    virtual int AnalogRead(uint pin) = 0;
    /**
     * @brief Register pin as PWM. This method is used for registered pins only. You should set a frequency and resolution before use.
     *       Attention! Not all drivers supports of PWM. You should check it before use
     *       Attention 2! Not all drivers supports of all frequencies and resolutions. You should check it before use
     * @param pin - number of pin
     * @param frequency - PWM frequency in Hz. The default value is 5000 Hz
     * @return true when the pin is registered as PWM
     */
    virtual bool RegisterPwmPin(uint pin, uint frequency = 5000) = 0;
    /**
     * @brief Unregister pin as PWM. This method is used for registered pins only.
     * @param pin - number of pin
     * @return true when the pin is unregistered as PWM
     */
    virtual bool UnRegisterPwmPin(uint pin) = 0;
    /**
     * @brief Set PWM value to the pin. This method is used for registered pins only.
     *        The value is set in percent (0...100%). The value is normalized to the resolution of PWM.
     * @param pin - number of pin
     * @param value - PWM value. The value should be in range from 0 to 100%. The value is normalized to the resolution of PWM.
     * @return true when the PWM value is set
     */
    virtual bool SetPwmPercent(uint pin, uint value) = 0;
    /**
     * @brief Set PWM value to the pin. This method is used for registered pins only.
     *        The value is set in microseconds.
     * @param pin - number of pin
     * @param value - PWM value in microseconds
     * @return true when the PWM value is set
     */
    virtual bool SetPwmRaw(uint pin, uint value) = 0;
    /**
     * @brief Set PWM value to the pin. This method is used for registered pins only.
     *        The value is set in microseconds.
     * @param pin - number of pin
     * @param value - PWM value in microseconds
     * @return true when the PWM value is set
     */
    virtual bool SetPwmUSec(uint pin, uint value) = 0;

    /**
     * @brief Get number of pins supported by the driver when it is a fixed number of pins. Returns 0 if the driver supports a variable number of pins.
     * @return number of pins
     */
    virtual uint GetNumberOfPins() = 0;

    /**
     * @brief Get PWM frequency. This method is used for registered pins only.
     * @param pin - number of pin 
     * @return PWM frequency in Hz or 0 if the pin is not registered as PWM
     */
    virtual uint GetPwmFrequency(uint pin) = 0;

    /**
     * @brief Get minimum/maximum PWM frequency. This method is used for registered pins only.
     * @param pin - number of pin 
     * @return minimum PWM frequency in Hz or 0 if the pin is not registered as PWM
     */
    virtual uint GetMinPwmFrequency(uint pin) = 0;
    virtual uint GetMaxPwmFrequency(uint pin) = 0;

    /**
     * @brief Get PWM resolution. This method is used for registered pins only.
     * @param pin - number of pin 
     * @return PWM resolution or 0 if the pin is not registered as PWM
     */
    virtual uint GetPwmResolution(uint pin) = 0;

    /**
     * @brief Get maximum PWM value. This method is used for registered pins only.
     * @param pin - number of pin 
     * @return maximum PWM value or 0 if the pin is not registered as PWM
     */
    virtual uint GetMaxPwmValue(uint pin) = 0;
    /**
     * @brief Get minimum PWM value. This method is used for registered pins only.
     * @param pin - number of pin 
     * @return minimum PWM value or 0 if the pin is not registered as PWM
     */
    virtual uint GetMinPwmValue(uint pin) = 0;


    /**
     * @brief This method is called after the driver is registered. You can use it to initialize the driver.
     */
    virtual void AfterRegistration(PinDriverDefinition pdd) = 0;

protected:
    static uint NormalizePwmValue(uint value, uint minValue, uint maxValue);
};