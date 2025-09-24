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
    SIGMAIO_ERROR_NOT_SUPPORTED,
    SIGMAIO_ERROR_BAD_BUS,
    SIGMAIO_ERROR_BUS_NOT_INITIALIZED,
    SIGMAIO_ERROR_BUS_NOT_FOUND
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

/*
typedef struct
{
    byte address;
    TwoWire *pWire;
    uint sda;
    uint scl;
    uint isrPin;
} I2CParams;
*/
typedef enum
{
    SIGMAIO_BUS_TYPE_NONE = 0,
    SIGMAIO_BUS_TYPE_GPIO,
    SIGMAIO_BUS_TYPE_I2C,
    SIGMAIO_BUS_TYPE_SPI,
    SIGMAIO_BUS_TYPE_UNKNOWN
} BusType;

typedef struct
{
    uint frequency;
    uint csPin;
    uint misoPin;
    uint mosiPin;
} SpiParams;

typedef struct
{
    uint frequency;
    uint sdaPin;
    uint sclPin;
} I2cParams;

typedef struct
{
    bool reserved;
} GpioParams;

typedef union
{
    SpiParams spiParams;
    I2cParams i2cParams;
    GpioParams gpioParams;
} BusParams;

typedef struct
{
    String name;
    BusType type = SIGMAIO_BUS_TYPE_UNKNOWN;
    uint busNumber;
    void *pBus; // pointer to the bus. It can be Wire, SPI, etc.
    BusParams busParams;
    //bool isInitialized = false;
} BusConfig;

typedef struct
{
    SigmaIoDriverCode driverCode;
    uint begin;
    uint end;
    void *pBus; // pointer to the bus. It can be Wire, SPI, etc. or nullptr for GPIO
    String busName;
    union
    {
        struct
        {
            uint scsPin;
        } spiParams;
        struct
        {
            uint address;
        } i2cParams;
    } busParams;
    union
    {
        struct
        {
            uint isrPin;
        } i2cDrvParams;
        struct
        {
            uint frequency;
        } pwmDrvParams;
    } driverParams;
} IODriverConfig;

typedef std::vector<IODriverConfig> IODriverSet;
typedef std::vector<BusConfig> BusSet;