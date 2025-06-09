#include <Arduino.h>
// #define PCF8575_DEBUG
#include "SigmaIO.h"
#include "SigmaPCF8575.h"
#include "SigmaGPIO.h"

/*
Hardware configuration:
    PLEASE NOTE: The PCF8575 can't pullup any pins. Please, pullup Interrupt pin and intput pins to VCC
    The PCF8575 port expander is connected to I2C bus
    The buttons are connected to the pin 0...7 of the PCF8575
    The LED's are connected to pins 8...15 of the PCF8575
    The interrupt pin is connected to the GPIO35

    Press any button to toggle correspondent LED (0;8), (1;9)...
*/

#define EXP_I2C_ADDRESS 0x27
#define EXP_ISR_PIN GPIO_NUM_39
#define EXP_INITIAL_PIN_ADDRESS 50
#define EXP_PIN_COUNT 16
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

void eventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    PinValue *pinValue = (PinValue *)event_data;
    // if (pinValue->pin >= EXP_INITIAL_PIN_ADDRESS && pinValue->pin < EXP_INITIAL_PIN_ADDRESS + 8)
    {
        if (event_id == SIGMAIO_EVENT_PIN1)
        {
            Serial.printf("1: Pin: %d, value: %d\n", pinValue->pin, pinValue->value);
        }
        else
        {
            Serial.printf("0: Pin: %d, value: %d\n", pinValue->pin, pinValue->value);
        }
        SigmaIO::DigitalWrite(pinValue->pin - EXP_INITIAL_PIN_ADDRESS + 8, pinValue->value);
    }
    // else
    //{
    //     Serial.printf("Some, not interested pin\n");
    // }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("--------------------------------------------");

    Wire.begin(I2C_SDA, I2C_SCL);
    esp_event_loop_create_default();

    IOError err = SIGMAIO_SUCCESS;

    SigmaPCF8575IO *pcf = new SigmaPCF8575IO(EXP_I2C_ADDRESS);
    err = SigmaIO::RegisterPinDriver(pcf, EXP_INITIAL_PIN_ADDRESS); // 50 - 65 is the range of maping the PCF8575 pins
    if (err != SIGMAIO_SUCCESS)
    {
        Serial.printf("PCF8575 registration error: %d\n", err);
        return;
    }
    err = SigmaIO::PinMode(EXP_ISR_PIN, INPUT_PULLUP);
    if (err != SIGMAIO_SUCCESS)
    {
        Serial.printf("ISR pin mode registration error: %d\n", err);
        return;
    }
    Serial.println("PCF8575 registered");
    esp_err_t errEsp = esp_event_handler_register_with(SigmaIO::GetEventLoop(), SigmaIO::GetEventBase(), SIGMAIO_EVENT_PIN, eventHandler, NULL);
    if (errEsp != ESP_OK)
    {
        Serial.printf("Event handler registration error: %d\n", errEsp);
        return;
    }
    errEsp = esp_event_handler_register_with(SigmaIO::GetEventLoop(), SigmaIO::GetEventBase(), SIGMAIO_EVENT_PIN1, eventHandler, NULL);
    if (errEsp != ESP_OK)
    {
        Serial.printf("Event handler1 registration error: %d\n", errEsp);
        return;
    }

    for (int i = 0; i < 8; i++)
    {
        err = SigmaIO::PinMode(i + EXP_INITIAL_PIN_ADDRESS, OUTPUT);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Led pin(%d) mode registration error: %d\n", (i + EXP_INITIAL_PIN_ADDRESS), err);
        }
        SigmaIO::DigitalWrite(i + EXP_INITIAL_PIN_ADDRESS, HIGH);
    }
    Serial.println("All output pins registered");
    for (int i = 8; i < 16; i++)
    {
        err = SigmaIO::PinMode(i + EXP_INITIAL_PIN_ADDRESS, INPUT);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Button pin(%d) mode registration error: %d\n", (i + EXP_INITIAL_PIN_ADDRESS), err);
        }
        err = SigmaIO::AttachInterrupt(EXP_ISR_PIN, i + EXP_INITIAL_PIN_ADDRESS, 200, FALLING);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Button pin(%d) interrupt registration error: %d\n", i, err);
        }
    }
    Serial.println("All input pins registered");
}

void loop()
{
    for (int i = 0; i < 8; i++)
    {
        SigmaIO::DigitalWrite(i + EXP_INITIAL_PIN_ADDRESS, LOW);
        delay(1000);
        SigmaIO::DigitalWrite(i + EXP_INITIAL_PIN_ADDRESS, HIGH);
        delay(1000);
    }
}