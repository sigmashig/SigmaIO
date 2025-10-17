#include <Arduino.h>
#include <SigmaIO.h>
#include <SigmaIOTypes.h>
#include <SigmaLoger.h>

#define EXP_I2C_ADDRESS 0x40
#define EXP_INITIAL_PIN_ADDRESS 60

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

#define PWM_PITCH 11 + EXP_INITIAL_PIN_ADDRESS
SigmaLoger *Log;

void setup()
{
  Serial.begin(115200);
  Serial.println("--------------------------------------------");
  IOError err = SIGMAIO_SUCCESS;
  Log = new SigmaLoger(512);
  Log->Append("TestBusDrivers.setup()").Debug();
  esp_event_loop_create_default();
  
  Serial.println("Event loop created");

  BusConfig busConfig;
  busConfig.type = SIGMAIO_BUS_TYPE_I2C;
  busConfig.name = "I2C";
  busConfig.busNumber = 0;
  busConfig.pBus = nullptr;
  busConfig.busParams.i2cParams.frequency = 200000;
  busConfig.busParams.i2cParams.sdaPin = I2C_SDA;
  busConfig.busParams.i2cParams.sclPin = I2C_SCL;
  err = SigmaIO::CreateBus(busConfig);
  if (err != SIGMAIO_SUCCESS)
  {
    Serial.printf("Bus creation error: %d\n", err);
    return;
  }

  IODriverConfig drvConfig;

  drvConfig.driverCode = SIGMAIO_PCA9685;
  drvConfig.begin = EXP_INITIAL_PIN_ADDRESS;
  drvConfig.end = EXP_INITIAL_PIN_ADDRESS + 15;
  drvConfig.busName = "I2C";
  drvConfig.busParams.i2cParams.address = EXP_I2C_ADDRESS;
  drvConfig.driverParams.pwmDrvParams.frequency = 100;
  
  err = SigmaIO::RegisterPinDriver(drvConfig, EXP_INITIAL_PIN_ADDRESS, 16);
  if (err != SIGMAIO_SUCCESS)
  {
    Serial.printf("Driver creation error: %d\n", err);
    return;
  }
  Serial.println("Driver created");
  SigmaIO::SetPwmUSec(PWM_PITCH, 600);
  Serial.println("PWM 600us set");
  delay(1000);
  SigmaIO::SetPwmUSec(PWM_PITCH, 2000);
  Serial.println("PWM 2000us set");
  delay(1000);
  SigmaIO::SetPwmUSec(PWM_PITCH, 800);
  Serial.println("PWM 800us set");

}

void loop()
{
  vTaskDelete(NULL);
}