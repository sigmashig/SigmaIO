#include <Arduino.h>
#include <PCF8575.h>
#include <Wire.h>

#define I2C_ADDRESS 0x27

PCF8575 expander(I2C_ADDRESS); // Default I2C address

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting...");
    //Wire.begin();
    Serial.println("Wire started");
    expander.begin();
    for (int i = 0; i < 8; i++)
    {
        expander.pinMode(i, OUTPUT);
        expander.digitalWrite(i, HIGH);
    }
    exp
    expander.digitalWrite(8, HIGH);
    expander.digitalWrite(9, HIGH);
    expander.digitalWrite(10, HIGH);
    expander.digitalWrite(11, HIGH);
    expander.digitalWrite(12, HIGH);
    expander.digitalWrite(13, HIGH);
    expander.digitalWrite(14, HIGH);
    expander.digitalWrite(15, HIGH);
    delay(2000);
    Serial.println("PCF8575 initialized");
    // expander.pinMode(0, OUTPUT); // Set pin 0 as output
}

void loop()
{

    for (int i = 0; i < 8; i++)
    {
        expander.digitalWrite(i, HIGH);
        delay(2500);
        expander.digitalWrite(i, LOW);
        delay(2500);
        Serial.printf("Pin %d\n", i);
    }
}
/*
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Serial.println("Wire started");
    }

void loop()
{
    int res;
    Serial.println("Transmission started");
    Wire.beginTransmission(I2C_ADDRESS);
    res = Wire.write((byte)0xFF);
    Serial.printf("res: %d\n", res);
    res = Wire.write((byte)0xFF);
    Serial.printf("res: %d\n", res);
    res = Wire.endTransmission();
    Serial.printf("END res: %d\n", res);
    Serial.println("Transmission ended");
    delay(1000);
    Serial.println("Transmission started2");
    Wire.beginTransmission(I2C_ADDRESS);
    res = Wire.write((byte)0x00);
    Serial.printf("res: %d\n", res);
    res = Wire.write((byte)0x00);
    Serial.printf("res: %d\n", res);
    res = Wire.endTransmission();
    Serial.printf("END2 res: %d\n", res);
    Serial.println("Transmission ended2");
    delay(3000);
}
*/