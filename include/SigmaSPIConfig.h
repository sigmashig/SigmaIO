#pragma once

#include <Arduino.h>

extern "C"
{
#include "driver/spi_master.h"
}

// SPI configuration used across modules
typedef struct
{
    spi_host_device_t spiHost;
    int misoPin;
    int mosiPin;
    int sckPin;
    int csPin;
    uint spiClockMHz;
    int queueSize;
    spi_device_interface_config_t devcfg;
} SPIConfig;
