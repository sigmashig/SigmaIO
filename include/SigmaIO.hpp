#pragma once
#include <Arduino.h>
#include "SigmaAbstractPinDriver.hpp"
// #include <vector>
#include <map>
#include <esp_event.h>
#include <SigmaGPIO.hpp>
#include <Wire.h>

ESP_EVENT_DECLARE_BASE(SIGMAIO_EVENT);

typedef enum
{
    SIGMAIO_GPIO = 0x00,
    SIGMAIO_PCF8575
} SigmaIoDriver;

typedef enum
{
    SIGMAIO_EVENT_DIRTY = 0,
    SIGMAIO_EVENT_PIN
} SigmaIoEvent;

typedef enum
{
    SIGMAIO_SUCCESS = 0,
    SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED,
    SIGMAIO_ERROR_PIN_NOT_REGISTERED,
    SIGMAIO_ERROR_PIN_DRIVER_ALREADY_REGISTERED,
    SIGMAIO_ERROR_BAD_PIN_RANGE,
    SIGMAIO_ERROR_BAD_PIN_DRIVER,
    SIGMAIO_ERROR_BAD_DRIVER_CODE,
    SIGMAIO_ERROR_BAD_DRIVER_PARAMS,
    SIGMAIO_ERROR_PIN_NOT_PWM,
    SIGMAIO_ERROR_INTERRUPT_ALREADY_ATTACHED,
    SIGMAIO_ERROR_INTERRUPT_NOT_ATTACHED
} IOError;

typedef struct
{
    uint pin;
    byte value;
    bool isTimerActive;
    uint debounceTime;
    TimerHandle_t timer;
} PinValue;

typedef struct
{
    uint pinIsr;
    std::map<uint, PinValue *> pinSrcMap;
} InterruptDescription;

typedef struct
{
    uint beg;
    uint end;
    bool isInternal;
    SigmaAbstractPinDriver *pinDriver;
} PinDriverDefinition;

typedef struct
{
    byte address;
    TwoWire *pWire;
    uint sda;
    uint scl;
} I2CParams;

class SigmaIO
{
public:
    SigmaIO(bool isRegisterGPIO = true);
    ~SigmaIO();
    void Begin();
    IOError DetachInterruptAll(uint pinIsr);
    IOError PinMode(uint pin, byte mode);
    void DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin);
    IOError RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd);
    IOError RegisterPinDriver(SigmaIoDriver driverCode, void *drvParams, byte pinBegin, byte pinEnd);
    IOError UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver);
    IOError RegisterPwmPin(uint pin, uint frequency = 5000, byte resolution = 7, uint minValue = 0, uint maxValue = 0xFFFF);
    IOError SetPwm(uint pin, uint value);
    PinDriverDefinition GetPinDriver(uint pin);
    // Interrupts
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

    IOError AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime = 0, int mode = FALLING);
    IOError DetachInterrupt(uint pinIsr, uint pinSrc);

private:
    /**
     * @brief The map of pin ranges and drivers. The key is the BEGIN of pin range. The value is the driver
     * Contains one record for the one driver.
     */
    std::map<uint, PinDriverDefinition> pinRangeDriverSet;
    /**
     * @brief The map of pins and drivers. The key is the pin number. The value is the driver. Every pin registered in the system are added to this map
     */
    std::map<uint, PinDriverDefinition> pinDriverSet;
    IOError checkDriverRegistrationAbility(byte pinBegin, byte pinEnd);

    static void processISR(void *arg);
    static std::map<uint, InterruptDescription *> interruptMap;
    static void processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void checkDebounced(TimerHandle_t xTimer);
};

extern SigmaIO *sigmaIO;