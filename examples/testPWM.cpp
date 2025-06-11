#include <Arduino.h>
#include <SigmaIO.h>

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting...");
    IOError res = SIGMAIO_SUCCESS;
    Serial.println("--------------------------------");
    res = SigmaIO::RegisterPwmPin(32);
    if (res != SIGMAIO_SUCCESS)
    {
        Serial.printf("Error registering PWM pin: %d\n", res);
    }
    else
    {
        Serial.println("PWM pin registered");
    }
    SigmaIO::SetPwm(32, 0);
    SigmaIO::SetPwm(32, 100);
    SigmaIO::SetPwm(32, 50);
    SigmaIO::SetPwm(32, 75);
}

void loop()
{
    vTaskDelete(NULL);
}
