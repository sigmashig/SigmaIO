#pragma once
#include <Arduino.h>
#include "SigmaAbstractPinDriver.hpp"
// #include <vector>
#include <map>
#include <esp_event.h>
#include <SigmaGPIO.hpp>

ESP_EVENT_DECLARE_BASE(SIGMAIO_EVENT);

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

class SigmaIO
{
public:
    SigmaIO();
    ~SigmaIO();
    IOError DetachInterruptAll(uint pinIsr);
    IOError PinMode(uint pin, byte mode);
    void DigitalWrite(uint pin, byte value);
    byte DigitalRead(uint pin);
    IOError RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd);
    IOError UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver);
    IOError RegisterPwmPin(uint pin, uint frequency = 5000, byte resolution = 7, uint minValue = 0, uint maxValue = 0xFFFF);
    IOError SetPwm(uint pin, uint value);
    // Interrupts
    IOError AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime = 0, int mode = FALLING);
    IOError DetachInterrupt(uint pinIsr, uint pinSrc);
    volatile int isrCnt = 0;
    volatile esp_err_t err;
    volatile byte p;

private:
    SigmaGPIO *gpio;
    typedef struct
    {
        uint beg;
        uint end;
        SigmaAbstractPinDriver *pinDriver;
    } PinDriverDefinition;
    std::map<uint, PinDriverDefinition> pinRangeDriverSet;
    std::map<uint, PinDriverDefinition> pinDriverSet;
    PinDriverDefinition getPinDriver(uint pin);
    /*
    struct cmpByInterruptKey
    {
        bool operator()(const InterruptKey &a, const InterruptKey &b) const
        {
            return (a.pinIsr < b.pinIsr);}
    };
    */
    static void processISR(void *arg);
    static std::map<uint, InterruptDescription *> interruptMap;
    static void processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void checkDebounced(TimerHandle_t xTimer);
};

extern SigmaIO *sigmaIO;