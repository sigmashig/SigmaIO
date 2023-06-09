#pragma once
#include <Arduino.h>

class SigmaAbstractPinDriver
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
    virtual void PinMode(byte pin, byte mode) = 0;
/**
 * @brief Write digital value to the pin. This method is used for registered pins only.
*/
    virtual void DigitalWrite(byte pin, byte value) = 0;
    /**
     * @brief Read digital value from the pin. This method is used for registered pins only.
    */
    virtual byte DigitalRead(byte pin) = 0;
    /**
     * @brief Check if pin can be used as PWM. This method is used for registered pins only.
     * @return true when the PWM is supported
    */
    virtual bool CanBePWM(byte pin) = 0;
    /**
     * @brief Write analog value to the pin. This method is used for registered pins only.
     *        Attention! Not all drivers supports of analog write. You should check it before use
    */
    virtual void AnalogWrite(byte pin, uint value) = 0;
    /**
     * @brief Read analog value from the pin. This method is used for registered pins only.
     *        Attention! Not all drivers supports of analog read. You should check it before use
     */
    virtual int AnalogRead(byte pin) = 0;
    /**
     * @brief Register pin as PWM. This method is used for registered pins only. You should set a frequency and resolution before use.
     *       Attention! Not all drivers supports of PWM. You should check it before use
     *       Attention 2! Not all drivers supports of all frequencies and resolutions. You should check it before use
     * @param pin - number of pin
     * @param frequency - PWM frequency in Hz. The default value is 5000 Hz
     * @param resolution - PWM resolution. The default value is 7 bits
     * @param minValue - minimum value for PWM. The default value is 0. It is a minimal value which set to the pin when 0% is requested.
     * @param maxValue - maximum value for PWM. The default value is 0xFFFF. It is a maximal value which set to the pin when 100% is requested.
     * @return true when the pin is registered as PWM
    */
    virtual bool RegisterPwmPin(byte pin, uint frequency=5000, byte resolution=7, uint minValue = 0, uint maxValue = 0xFFFF) = 0;
    /**
     * @brief Unregister pin as PWM. This method is used for registered pins only.
     * @param pin - number of pin
     * @return true when the pin is unregistered as PWM
    */
    virtual bool UnRegisterPwmPin(byte pin) = 0;
    /**
     * @brief Set PWM value to the pin. This method is used for registered pins only.
     *        The value is set in percent (0...100%). The value is normalized to the resolution of PWM.
     * @param pin - number of pin
     * @param value - PWM value. The value should be in range from 0 to 100%. The value is normalized to the resolution of PWM.
     * @return true when the PWM value is set
    */
    virtual bool SetPwm(byte pin, uint value) = 0;
    /**
     * @brief Begin method. This method is used for initialization of the driver, when it is needed. It calls in Begin Method of the IO system.
     *        The method should be called before any other method of the driver.
     * @return true when the driver is initialized
    */
    virtual bool Begin() = 0;

protected:
    static uint NormalizePwmValue(uint value, byte resolution, uint minValue = 0, uint maxValue = 0xFFFF);
};