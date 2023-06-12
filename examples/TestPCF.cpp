#include <Arduino.h>
#include "SigmaIO.hpp"
#include "SigmaPCF8575.hpp"
#include "SigmaGPIO.hpp"

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
#define EXP_ISR_PIN GPIO_NUM_35
#define EXP_INITIAL_PIN_ADDRESS 50
#define EXP_PIN_COUNT 16

void eventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    PinValue *pinValue = (PinValue *)event_data;
    if (pinValue->pin >= EXP_INITIAL_PIN_ADDRESS && pinValue->pin < EXP_INITIAL_PIN_ADDRESS + 8)
    {
        Serial.printf("Pin: %d, value: %d\n", pinValue->pin, pinValue->value);
        sigmaIO->DigitalWrite(pinValue->pin - EXP_INITIAL_PIN_ADDRESS + 8, pinValue->value);
    }
    else
    {
        Serial.printf("Some, not interested pin\n");
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("--------------------------------------------");

    esp_event_loop_create_default();

    IOError err = SIGMAIO_SUCCESS;
    sigmaIO = new SigmaIO();
    Serial.println("SigmaIO created");

    SigmaPCF8575IO *pcf = new SigmaPCF8575IO(EXP_I2C_ADDRESS);
    err = sigmaIO->RegisterPinDriver(pcf, 50, 65); // 50 - 65 is the range of maping the PCF8575 pins
    if (err != SIGMAIO_SUCCESS)
    {
        Serial.printf("PCF8575 registration error: %d\n", err);
        // while(1);
    }
    err = sigmaIO->PinMode(EXP_ISR_PIN, INPUT_PULLUP);
    if (err != SIGMAIO_SUCCESS)
    {
        Serial.printf("ISR pin mode registration error: %d\n", err);
    }
    Serial.println("PCF8575 registered");
    esp_event_handler_register(SIGMAIO_EVENT, SIGMAIO_EVENT_PIN, eventHandler, NULL);

    for (int i = 0; i < 8; i++)
    {
        err = sigmaIO->PinMode(i + 8 + EXP_INITIAL_PIN_ADDRESS, OUTPUT);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Led pin(%d) mode registration error: %d\n", (i + 8), err);
        }
        sigmaIO->DigitalWrite(i + EXP_INITIAL_PIN_ADDRESS, HIGH);

        err = sigmaIO->PinMode(i + EXP_INITIAL_PIN_ADDRESS, INPUT);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Button pin(%d) mode registration error: %d\n", i, err);
        }
        err = sigmaIO->AttachInterrupt(EXP_ISR_PIN, i + EXP_INITIAL_PIN_ADDRESS, 100, FALLING);
        if (err != SIGMAIO_SUCCESS)
        {
            Serial.printf("Button pin(%d) interrupt registration error: %d\n", i, err);
        }
    }
    Serial.println("Buttons and LEDs registered");
}

void loop()
{
    /*
    static uint step = 0;
    static bool v = true;
    ;
    Serial.printf("Step: %d, value: %d\n", step, v);
    //sigmaIO->DigitalWrite(step + 8 + EXP_INITIAL_PIN_ADDRESS, v);

    for (int i = 0; i < 8; i++)
    {
        bool btn = sigmaIO->DigitalRead(i + EXP_INITIAL_PIN_ADDRESS);
        Serial.printf("Button %d: %d\n", i, btn);
        sigmaIO->DigitalWrite(i + 8 + EXP_INITIAL_PIN_ADDRESS, btn);
    }

    delay(1000);

    step++;
    if (step == 8)
    {
        v = !v;
        step = 0;
    }
    */
}