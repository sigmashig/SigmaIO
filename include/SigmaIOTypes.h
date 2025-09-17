#pragma once
#include <Arduino.h>
#include <map>
#include <Wire.h>

class SigmaAbstractPinDriver;
class SigmaIODriver;

typedef struct PinDriverDefinition
{
    uint beg;
    uint end;
    bool isInternal;
    SigmaIODriver *pinDriver;
    uint pinMode;
} PinDriverDefinition;

typedef enum
{
    SIGMAIO_GPIO = 0x00,
    SIGMAIO_PCF8575,
    SIGMAIO_PCA9685,
    SIGMAIO_UNKNOWN
} SigmaIoDriverCode;

typedef enum
{
    SIGMAIO_EVENT_DIRTY = 0,
    SIGMAIO_EVENT_PIN,
    SIGMAIO_EVENT_PIN1
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
    SIGMAIO_ERROR_INTERRUPT_NOT_ATTACHED,
    SIGMAIO_ERROR_BAD_VALUE,
    SIGMAIO_WARNING_BAD_PWM_VALUE,
    SIGMAIO_ERROR_NOT_SUPPORTED
} IOError;

typedef struct
{
    uint pin;
    uint value;
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
    byte address;
    TwoWire *pWire;
    uint sda;
    uint scl;
    uint isrPin;
} I2CParams;

typedef struct
{
    //String name;
    SigmaIoDriverCode driverCode;
    uint begin;
    uint end;
    union
    {
        byte reserved;
        I2CParams i2cParams;
    } params;
    union 
    {
        byte reserved;
        struct {
            uint frequency;
        } pwmParams;
    } driverParams;
    
} IODriverConfig;

typedef std::vector<IODriverConfig> IODriverSet;