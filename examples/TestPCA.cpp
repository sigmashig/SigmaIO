#include <Arduino.h>
// #define PCF8575_DEBUG
#include "SigmaIO.h"
#include "SigmaPCA9685.h"
#include "SigmaGPIO.h"


#define EXP_I2C_ADDRESS 0x40
#define EXP_INITIAL_PIN_ADDRESS 60

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22


void setup()
{
    Serial.begin(115200);
    Serial.println("--------------------------------------------");

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Wire initialized");
    esp_event_loop_create_default();

    IOError err = SIGMAIO_SUCCESS;
    Serial.println("Event loop created");

    SigmaPCA9685IO *expanded = new SigmaPCA9685IO(EXP_I2C_ADDRESS);
    Serial.println("PCA9685 created");
    err = SigmaIO::RegisterPinDriver(expanded, EXP_INITIAL_PIN_ADDRESS); // 60 - 75 is the range of maping the PCF8575 pins
    if (err != SIGMAIO_SUCCESS)
    {
        Serial.printf("PCA9685 registration error: %d\n", err);
        return;
    }
    Serial.println("PCA9685 registered");

    SigmaIO::SetPwm(12+EXP_INITIAL_PIN_ADDRESS, 100);
    SigmaIO::SetPwm(15+EXP_INITIAL_PIN_ADDRESS, 0);
    
}

void loop()
{
    Serial.println("First pass");
    for (int i=0;i<10;i++)
    {
        SigmaIO::SetPwm(12+EXP_INITIAL_PIN_ADDRESS, i*10);
        SigmaIO::SetPwm(15+EXP_INITIAL_PIN_ADDRESS, 100-i*10);
        delay(500);
    }
    Serial.println("Second pass");
    for (int i=10;i>0;i--)
    {
        SigmaIO::SetPwm(12+EXP_INITIAL_PIN_ADDRESS, i*10);
        SigmaIO::SetPwm(15+EXP_INITIAL_PIN_ADDRESS, 100-i*10);
        delay(500);
    }
}