# SigmaIoAbstraction
* FOR ESP32 ONLY!!! *
Universal IO Factory which gather a different drivers GPIO, Port Expander, Multiplexor in one interface
The main idea: you are register a differen drivers in the system and then you use it via common interface.
## Usage
### Step 1: Create an Instance
Create an instance of IO system. You can use own variable or use *sigmaIO declared in the library.
```
sigmaIO = new SigmaIO(true);
```
The parameter means:
* true (default) - automatically register GPIO driver at range 0...GPIO_MAX-1
If you need to register own port expander, you should go Step 2. Otherwise, go to Step 4.
### Step 2: (optional). Create own class driver
Create own driver as descendant of SigmaIOAbstractDriver. If you are going to use some standard drivers delivered in the library, 
you can skip this step.
```
class MyDriver : public SigmaIOAbstractDriver {...}
...
```
### Step 3: (required). Register driver in the system
Register driver in the system. This step you can use own driver or standard driver from the library.
```
SigmaPCF8575 *pcf = new SigmaPCF8575(0x20);
sigmaIO->registerDriver(pcf, 50, 65);

MyDriver *myDriver = new MyDriver(0x15, "http://example.com");
sigmaIO->registerDriver(myDriver, 100, 105);
```
As alternative way, you can use the following code:
```
I2CParams i2cParams = {0x20, NULL, 0, 0}; // address, pointer to the I2C bus, SDA, SCL. Address only is used when pWire is NULL 

sigmaIO->registerDriver(SIGMAIO_PCF, &i2cParams, 50, 65);
```
this case, the driver will be created automatically. The second parameter is void* is pointer to the driver constructor. 
You can get a poiinter to the driver by calling GetPinDriver method. Example:
```
SigmaPCF8575 *pcf = (SigmaPCF8575 *)sigmaIO->GetPinDriver(55);

```

The registration map the range Begin...End to the driver. In the example above, the range 50...65 will be mapped to the PCF8575 driver and the range 100...105 will be mapped to the MyDriver driver. The range is used for the driver selection. 
The Begin will be subtracted when call a function like pinMode, digitalRead, digitalWrite, analogRead, analogWrite. Example:
```
sigmaIO->digitalWrite(55, HIGH); // call PCF8575 driver with pin 55, but the driver will be called with pin 5 (55-50)
```
### Step 4. Begin
You should call Begin method to initialize the system. Example:
```
sigmaIO->Begin();
```

### Step 5: Required. Pin Registration
Before use any pin, you should register it. Example:
```
sigmaIO->pinMode(55, OUTPUT);
```
You can change the pin mode later. But you should register the pin before use it.

### Step 6: Optional. Using the PWM

First of all, you should register pin as PWM pin. This operation creates a chennel and set parameters. Example:
```
    IOError err = RegisterPwmPin(55, 5000, 7, 34, 120);
```
Some explanation about parameters:
* 55 - pin number. Please, check, that this pin mapped to driver supports PWM
* 5000 - frequency in Hz, Please, check the driver supports this frequency
* 7 - resolution, Please, check the driver supports this resolution
* 34, 120 - minimal and maximum values. These parameters will be set into PWM port, when you select 0% or 100% correspondently 


### Step 7: Required. Use the pin
```
byte b = sigmaIO->digitalWrite(100);
IOError err = sigmaIO->SetPwm(55, 50); // set 50% duty cycle
```


## Standard Drivers
### SigmaGPIO - classical GPIO driver
This driver is registered automatically (when you call constructor with parameter true) or manually.
### SigmaPCF8575 - PCF8575 port expander - I2C 16 port expander. You should set the I2C address


# History
* v.0.1.1 - 2023/06/13 - Initial release
* v.0.1.2 - 2023/06/14 - Add Begin method. Fix errors
* v.0.1.3 - 2023/06/14 - Add autocreation of PCF driver