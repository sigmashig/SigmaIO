#include <Arduino.h>
#include <Wire.h>

void scanI2CBus()
{
    byte error, address;
    int deviceCount = 0;

    Serial.println("\n----- Scanning I2C bus -----");

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (address % 10 == 0)
        {
            Serial.printf("Scanned addresses: 0x%02X\n", address);
        }
        if (error == 0)
        {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            deviceCount++;
        }
        else if (error == 4)
        {
            Serial.printf("Unknown error at address 0x%02X\n", address);
        }
    }

    if (deviceCount == 0)
    {
        Serial.println("No I2C devices found");
    }
    else
    {
        Serial.printf("Found %d device(s) on I2C bus\n", deviceCount);
    }
    Serial.println("--------------------------------------------");
}

void setup()
{
    Serial.begin(115200);
    delay(1000); // Give serial monitor time to open

    Serial.println("\n\nI2C Scanner starting...");

    // Initialize I2C
    Wire.begin();

    // Perform initial scan
    scanI2CBus();
}

void loop()
{
    vTaskDelete(NULL);
}