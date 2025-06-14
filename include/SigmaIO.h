#pragma once
#include <Arduino.h>
#include <map>
#include <esp_event.h>
#include <SigmaGPIO.h>
#include <Wire.h>
#include "SigmaIODriver.h"

// PinDriverDefinition is now defined in SigmaIOTypes.h

class SigmaIO
{
public:
    // SigmaIO(bool isRegisterGPIO = true);
    //~SigmaIO();
    // void Begin();
    /**
     * @brief Register pin driver for the pin range. Both pinBegin and pinEnd are included to the range
     * @param pinDriver - pointer to the driver
     * @param pinBegin - begin of the pin range
     * @param numberPins - number of the pins (or 0 if driver supports a fixed numbers of pins)
     */
    static IOError RegisterPinDriver(SigmaIODriver *pinDriver, uint pinBegin, uint numberPins = 0);
    /**
     * @brief Register pin driver for the pin range. Both pinBegin and pinEnd are included to the range.
     *       This method is used for internal drivers only (GPIO, PCF8575)
     * @param driverCode - driver code
     * @param drvParams - pointer to the driver parameters. The type of parameters depends on the driver code
     * @param numberPins - number of the pins (or 0 if driver supports a fixed numbers of pins)
     */
    static IOError RegisterPinDriver(SigmaIoDriver driverCode, IODriverConfig drvConfig, uint pinBegin, uint numberPins = 0);
    static IOError UnregisterPinDriver(SigmaIODriver *pinDriver);
    static PinDriverDefinition GetPinDriver(uint pin);
    static void Create(IODriverSet ioConfigs);

    static IOError PinMode(uint pin, byte mode);
    static IOError DigitalWrite(uint pin, byte value);
    static byte DigitalRead(uint pin);

    static uint AnalogRead(uint pin);
    static IOError AnalogWrite(uint pin, uint value);

    /**
     * @brief Register PWM pin. This method is used when pin supports an individual PWM. The pin should be registered as output pin before using this method
     * @param pin - pin number
     * @param frequency - frequency in Hz
     */
    static IOError RegisterPwmPin(uint pin, uint frequency = 5000);
    // Interrupts
    static IOError SetPwm(uint pin, uint value);
    /**
     * @brief Attach interrupt to the pin. This method is used for registered pins only. You should register both pins: ISR and SRC
     *          pins before using. You can attach a several pins to one interrupt. This case, the event will be posted for every pin,
     *          which change the value since last interrupt
     * @param pinIsr - pin number for interrupt
     * @param pinSrc - pin number for source
     * @param debounceTime - debounce time in milliseconds. If 0, the debounce will be disabled. The debounce time is set individually for each pin
     * @param mode - interrupt mode. See Arduino attachInterrupt() function for details. Attention! Not all modes are supported by all drivers.
     *         The CHANGE, RISING, FALLING should be supported by all drivers. Support of LOW, HIGH etc is not guarantied.
     *
     */

    static IOError AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime = 0, int mode = FALLING);
    static IOError DetachInterrupt(uint pinIsr, uint pinSrc);
    static IOError DetachInterruptAll(uint pinIsr);

    static esp_err_t SetEventLoop(esp_event_loop_handle_t _eventLoop);
    static esp_event_loop_handle_t GetEventLoop() { return eventLoop; }
    static esp_event_base_t GetEventBase() { return eventBase; }

private:
    /**
     * @brief The map of pin ranges and drivers. The key is the BEGIN of pin range. The value is the driver
     * Contains one record for the one driver.
     */
    inline static std::map<uint, PinDriverDefinition> pinRangeDriverSet;
    /**
     * @brief The map of pins and drivers. The key is the pin number. The value is the driver. Every pin registered in the system are added to this map
     */
    inline static std::map<uint, PinDriverDefinition> pinDriverSet;
    inline static esp_event_base_t eventBase = "SigmaIO";
    inline static esp_event_loop_handle_t eventLoop = NULL;
    inline static bool isInit = false;
    inline static std::map<uint, InterruptDescription *> interruptMap;

    static IOError checkDriverRegistrationAbility(uint pinBegin, uint numberPins);
    static void processISR(void *arg);
    static void processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void checkDebounced(TimerHandle_t xTimer);
    static void init();
    static SigmaIoDriver driverName2Type(String driverName);
};
