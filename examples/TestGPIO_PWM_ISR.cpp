#include <Arduino.h>
#include "SigmaIO.hpp"
#include "SigmaGPIO.hpp"
#include <esp_event.h>

#define LED_PIN LED_BUILTIN
#define BUTTON_PIN 26 // PIN where button is connected
#define ISR_PIN 14    // PIN where ISR is connected. Please, pay attention, that ISR_PIN CAN be different from BUTTON_PIN

uint power = 0;

void eventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  switch (event_id)
  {
  case SIGMAIO_EVENT_PIN:
  {
    Serial.println("Pin event");
    PinValue *pinValue = (PinValue *)event_data;
    Serial.printf("Pin: %d, value: %d\n", pinValue->pin, pinValue->value);
    if (pinValue->pin == BUTTON_PIN && pinValue->value == LOW)
    {
      power += 9;
      if (power > 99)
      {
        power = 0;
      }
      Serial.printf("Power: %d\n", power);
      sigmaIO->SetPwm(LED_PIN, power);
    }
    break;
  }
  default:
    Serial.println("Some, not interested event");
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("--------------------------------------------");
  Serial.println("SigmaIO Test");
  Serial.println("--------------------------------------------");
  Serial.println("When you click on button, the LED will increase its brightness");

  esp_event_loop_create_default();

  IOError err = SIGMAIO_SUCCESS;
  sigmaIO = new SigmaIO();
  Serial.println("SigmaIO created");

  err = sigmaIO->PinMode(LED_PIN, OUTPUT);
  if (err != SIGMAIO_SUCCESS)
  {
    Serial.printf("Led pin mode registration error: %d\n", err);
  }
  err = sigmaIO->PinMode(BUTTON_PIN, INPUT_PULLUP);
  if (err != SIGMAIO_SUCCESS)
  {
    Serial.printf("Button pin mode registration error: %d\n", err);
  }
  sigmaIO->RegisterPwmPin(LED_PIN, 5000, 8);
  sigmaIO->SetPwm(LED_PIN, 0);
  sigmaIO->PinMode(ISR_PIN, INPUT_PULLUP);
  sigmaIO->AttachInterrupt(ISR_PIN, BUTTON_PIN, 100, FALLING);
  Serial.println("Pin drivers registered");
  esp_event_handler_register(SIGMAIO_EVENT, ESP_EVENT_ANY_ID, eventHandler, NULL);
}

void loop()
{
  delay(100);
}
