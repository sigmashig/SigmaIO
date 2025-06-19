#include "SigmaIO.h"
#include "SigmaGPIO.h"
#include "SigmaPCF8575.h"
#include "SigmaPCA9685.h"

void SigmaIO::init()
{
    isInit = true;
    IODriverConfig drvConfig;
    RegisterPinDriver(SIGMAIO_GPIO, drvConfig, 0, GPIO_PIN_COUNT);
    if (eventLoop == NULL)
    {
        esp_event_loop_args_t loop_args = {
            .queue_size = (int32_t)100,
            .task_name = "SigmaIO",
            .task_priority = 50,
            .task_stack_size = 4096,
            .task_core_id = 0};

        esp_event_loop_create(&loop_args, (void **)&eventLoop);
        if (eventLoop == NULL)
        {
            Serial.println("SigmaIO: event loop creation failed");
        }
    }
    esp_event_handler_register_with(eventLoop, eventBase, SIGMAIO_EVENT_DIRTY, processInterrupt, NULL);
}

SigmaIoDriver SigmaIO::DriverName2Type(String driverName)
{
    if (driverName == "GPIO")
    {
        return SIGMAIO_GPIO;
    }
    else if (driverName == "PCF8575")
    {
        return SIGMAIO_PCF8575;
    }
    else if (driverName == "PCA9685")
    {
        return SIGMAIO_PCA9685;
    }
    return SIGMAIO_UNKNOWN;
}

uint SigmaIO::GetNumberOfPins(SigmaIoDriver driverCode)
{
    if (driverCode == SIGMAIO_GPIO)
    {
        return GPIO_PIN_COUNT;
    }
    else if (driverCode == SIGMAIO_PCF8575)
    {
        return 16;
    }
    else if (driverCode == SIGMAIO_PCA9685)
    {
        return 16;
    }
    return 0;
}

IOError SigmaIO::DetachInterruptAll(uint pinIsr)
{
    for (auto itIsr : interruptMap)
    {
        if (itIsr.first == pinIsr)
        {
            for (auto itSrc : itIsr.second->pinSrcMap)
            {
                DetachInterrupt(pinIsr, itSrc.first);
            }
        }
    }
    return SIGMAIO_SUCCESS;
}

esp_err_t SigmaIO::SetEventLoop(esp_event_loop_handle_t _eventLoop)
{
    if (_eventLoop == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    //    if (eventLoop != NULL)
    //    {
    //        esp_event_handler_unregister_with(eventLoop, eventBase, SIGMAIO_EVENT_DIRTY, &processInterrupt);
    //        esp_event_loop_delete(eventLoop);
    //    }

    esp_event_handler_unregister_with(eventLoop, eventBase, SIGMAIO_EVENT_DIRTY, &processInterrupt);
    eventLoop = _eventLoop;
    esp_event_handler_register_with(_eventLoop, eventBase, SIGMAIO_EVENT_DIRTY, &processInterrupt, NULL);
    return ESP_OK;
}

std::vector<byte> SigmaIO::ScanI2C()
{
    std::vector<byte> addresses;
    Wire.begin();
    addresses.clear();
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint error = Wire.endTransmission();
        if (error == 0)
        {
            addresses.push_back(address);
        }
    }
    return addresses;
}

IOError SigmaIO::PinMode(uint pin, byte mode)
{
    if (!isInit)
    {
        init();
    }
    PinDriverDefinition pdd = GetPinDriver(pin);
    IOError res = SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    if (pdd.pinDriver == nullptr)
    {
        return res;
    }

    // pin is not initialized yet
    auto it = pinDriverSet.find(pin);
    if (it == pinDriverSet.end())
    { // pin is not registered
        pinDriverSet.insert({pin, pdd});
    }
    pdd.pinMode = mode;
    pdd.pinDriver->PinMode(pin - pdd.beg, mode);
    return SIGMAIO_SUCCESS;
}

uint SigmaIO::GetPinMode(uint pin)
{
    for (auto it = pinDriverSet.begin(); it != pinDriverSet.end(); it++)
    {
        if (it->first == pin)
        {
            return it->second.pinMode;
        }
    }
    return 0;
}

IOError SigmaIO::DigitalWrite(uint pin, byte value)
{
    if (!isInit)
    {
        init();
    }
    IOError res = SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        pdd.pinDriver->DigitalWrite(pin - pdd.beg, value);
        res = SIGMAIO_SUCCESS;
    }
    return res;
}

byte SigmaIO::DigitalRead(uint pin)
{
    if (!isInit)
    {
        init();
    }
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        return pdd.pinDriver->DigitalRead(pin - pdd.beg);
    }
    return 0xFF;
}

IOError SigmaIO::AnalogWrite(uint pin, uint value)
{
    if (!isInit)
    {
        init();
    }
    IOError res = SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        pdd.pinDriver->AnalogWrite(pin - pdd.beg, value);
        res = SIGMAIO_SUCCESS;
    }
    return res;
}

uint SigmaIO::AnalogRead(uint pin)
{
    if (!isInit)
    {
        init();
    }
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        return pdd.pinDriver->AnalogRead(pin - pdd.beg);
    }
    return 0;
}

IOError SigmaIO::checkDriverRegistrationAbility(uint pinBegin, uint numberPins)
{
    if (numberPins == 0)
    {
        return SIGMAIO_ERROR_BAD_PIN_RANGE;
    }
    uint pinEnd = pinBegin + numberPins - 1;

    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        PinDriverDefinition pdd = it->second;
        if ((it->second.beg <= pinBegin && pinBegin <= it->second.end) || (it->second.beg <= pinEnd && pinEnd <= it->second.end))
        { // potentially found a hole. check if it is not overlapping with end
            return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
        }
    }
    return SIGMAIO_SUCCESS;
}
IOError SigmaIO::RegisterPinDriver(SigmaIoDriver driverCode, IODriverConfig drvConfig, uint pinBegin, uint numberPins)
{
    if (!isInit)
    {
        init();
    }
    if (numberPins == 0)
    {
        numberPins = GetNumberOfPins(driverCode);
    }

    IOError res = checkDriverRegistrationAbility(pinBegin, numberPins);
    if (res != SIGMAIO_SUCCESS)
    {
        return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
    }
    SigmaIODriver *pinDriver = nullptr;
    switch (driverCode)
    {
    case SIGMAIO_GPIO:
    {
        pinDriver = new SigmaGPIO();
        break;
    }
    case SIGMAIO_PCF8575:
    {
        I2CParams i2cParams = drvConfig.params.i2cParams;
        if (i2cParams.pWire == nullptr)
        {
            pinDriver = new SigmaPCF8575IO(i2cParams.address, i2cParams.isrPin);
        }
        else
        {
            pinDriver = new SigmaPCF8575IO(i2cParams.address, i2cParams.isrPin, i2cParams.pWire, i2cParams.sda, i2cParams.scl);
        }
        break;
    }
    case SIGMAIO_PCA9685:
    {
        I2CParams i2cParams = drvConfig.params.i2cParams;
        pinDriver = new SigmaPCA9685IO(i2cParams.address, 0, i2cParams.pWire);
        break;
    }
    default:
        return SIGMAIO_ERROR_BAD_DRIVER_CODE;
    }
    uint pinEnd = pinBegin + numberPins - 1;
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, true, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    pinDriver->AfterRegistration(newPair.second);

    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::RegisterPinDriver(SigmaIODriver *pinDriver, uint pinBegin, uint numberPins)
{
    if (!isInit)
    {
        init();
    }
    if (pinDriver == NULL)
    {
        return SIGMAIO_ERROR_BAD_PIN_DRIVER;
    }
    if (numberPins == 0)
    {
        numberPins = pinDriver->GetNumberOfPins();
    }
    IOError res = checkDriverRegistrationAbility(pinBegin, numberPins);
    if (res != SIGMAIO_SUCCESS)
    {
        return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
    }
    uint pinEnd = pinBegin + numberPins - 1;
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, false, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    pinDriver->AfterRegistration(newPair.second);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::UnregisterPinDriver(SigmaIODriver *pinDriver)
{
    if (!isInit)
    {
        init();
    }
    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        PinDriverDefinition pdd = it->second;
        if (pdd.pinDriver == pinDriver)
        {
            for (auto it1 = pinDriverSet.begin(); it1 != pinDriverSet.end(); it1++)
            {
                if (it1->second.pinDriver == pinDriver)
                {
                    pinDriverSet.erase(it1);
                }
            }
            pinRangeDriverSet.erase(it);
            return SIGMAIO_SUCCESS;
        }
    }

    return SIGMAIO_ERROR_BAD_PIN_DRIVER;
}

IOError SigmaIO::RegisterPwmPin(uint pin, uint frequency)
{
    if (!isInit)
    {
        init();
    }
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {

        if (pdd.pinDriver->CanBePWM(pin - pdd.beg))
        {
            if (pdd.pinDriver->RegisterPwmPin(pin - pdd.beg, frequency))
            {
                pinDriverSet.insert({pin, pdd});
                pdd.pinDriver->PinMode(pin - pdd.beg, OUTPUT);
                return SIGMAIO_SUCCESS;
            }
            else
            {
                return SIGMAIO_ERROR_PIN_NOT_PWM;
            }
        }
        else
        {
            return SIGMAIO_ERROR_PIN_NOT_PWM;
        }
    }
    else
    {
        return SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    }
}

IOError SigmaIO::SetPwm(uint pin, uint value)
{
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        if (pdd.pinDriver->SetPwm(pin - pdd.beg, value))
        {
            return SIGMAIO_SUCCESS;
        }
        else
        {
            return SIGMAIO_ERROR_PIN_NOT_PWM;
        }
    }
    else
    {
        return SIGMAIO_ERROR_PIN_NOT_REGISTERED;
    }
}

void SigmaIO::checkDebounced(TimerHandle_t xTimer)
{
    for (auto isrIt : interruptMap)
    {
        for (auto srcIt : isrIt.second->pinSrcMap)
        {
            if (srcIt.second->timer == xTimer)
            { // Timer found
                bool val = DigitalRead(srcIt.first);
                srcIt.second->isTimerActive = false;
                if (srcIt.second->value != val)
                { // The real input value has changed
                    srcIt.second->value = val;
                    esp_event_post_to(GetEventLoop(), GetEventBase(), SIGMAIO_EVENT_PIN1, srcIt.second, sizeof(PinValue), portMAX_DELAY);
                }
            }
        }
    }
}

IOError SigmaIO::AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime, int mode)
{
    if (!isInit)
    {
        init();
    }
    InterruptDescription *isrDescr = new InterruptDescription();
    auto it = interruptMap.find(pinIsr);
    if (it == interruptMap.end())
    {
        // New PinIsr

        isrDescr->pinIsr = pinIsr;
        PinValue *pv = new PinValue();
        pv->pin = pinSrc;
        pv->debounceTime = debounceTime;
        pv->value = DigitalRead(pinSrc);
        pv->isTimerActive = false;
        if (debounceTime != 0)
        {
            pv->timer = xTimerCreate("debounce", pdMS_TO_TICKS(pv->debounceTime), pdFALSE, NULL, checkDebounced);
        }
        else
        {
            pv->timer = NULL;
        }
        isrDescr->pinSrcMap.insert({pinSrc, pv});
        interruptMap.insert({pinIsr, isrDescr});
        attachInterruptArg(pinIsr, processISR, isrDescr, mode);
        return SIGMAIO_SUCCESS;
    }
    else
    {
        auto src = it->second->pinSrcMap.find(pinSrc);
        if (src != it->second->pinSrcMap.end())
        {
            // PinSrc already attached
            return SIGMAIO_ERROR_INTERRUPT_ALREADY_ATTACHED;
        }
        else
        {
            PinValue *pv = new PinValue();
            pv->pin = pinSrc;
            pv->value = DigitalRead(pinSrc);
            pv->debounceTime = debounceTime;
            pv->isTimerActive = false;
            if (debounceTime != 0)
            {
                pv->timer = xTimerCreate("debounce", pdMS_TO_TICKS(pv->debounceTime), pdFALSE, NULL, checkDebounced);
            }
            else
            {
                pv->timer = NULL;
            }
            isrDescr->pinSrcMap.insert({pinSrc, pv});
            it->second->pinSrcMap.insert({pinSrc, pv});
            return SIGMAIO_SUCCESS;
        }
    }
}

IOError SigmaIO::DetachInterrupt(uint pinIsr, uint pinSrc)
{
    if (!isInit)
    {
        init();
    }
    auto it = interruptMap.find(pinIsr);
    if (it == interruptMap.end())
    {
        // No PinIsr
        return SIGMAIO_ERROR_INTERRUPT_NOT_ATTACHED;
    }
    else
    {
        auto src = it->second->pinSrcMap.find(pinSrc);
        if (src != it->second->pinSrcMap.end())
        {
            // PinSrc attached
            if (src->second->timer != NULL)
            {
                xTimerDelete(src->second->timer, 0);
            }
            delete src->second;
            it->second->pinSrcMap.erase(src);
            if (it->second->pinSrcMap.size() == 0)
            {
                delete it->second;
                interruptMap.erase(it);
                detachInterrupt(pinIsr);
            }
            return SIGMAIO_SUCCESS;
        }
        else
        {
            return SIGMAIO_ERROR_INTERRUPT_NOT_ATTACHED;
        }
    }
}

PinDriverDefinition SigmaIO::GetPinDriver(uint pin)
{
    if (!isInit)
    {
        init();
    }
    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        if (it->second.beg <= pin && pin <= it->second.end)
        {
            return it->second;
        }
    }
    return {0, 0, false, nullptr};
}

void SigmaIO::Create(IODriverSet ioConfigs)
{
    for (auto &ioCfg : ioConfigs)
    {
        SigmaIoDriver driverCode = DriverName2Type(ioCfg.name);
        if (driverCode != SIGMAIO_UNKNOWN)
        {
            RegisterPinDriver(driverCode, ioCfg, ioCfg.begin);
        }
        else
        {
            Serial.println("SigmaIO: unknown driver name: " + ioCfg.name);
        }
    }
}

ICACHE_RAM_ATTR void SigmaIO::processISR(void *arg)
{
    InterruptDescription *descr = (InterruptDescription *)arg;
    esp_event_isr_post_to(GetEventLoop(), GetEventBase(), SIGMAIO_EVENT_DIRTY, &(descr->pinIsr), sizeof(descr->pinIsr), NULL);
}

void SigmaIO::processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint *pin = (uint *)event_data;
    auto itIsr = interruptMap.find(*pin);
    if (itIsr != interruptMap.end())
    {
        for (auto itSrc : itIsr->second->pinSrcMap)
        {
            bool val = DigitalRead(itSrc.first);
            if (itSrc.second->value != val)
            {
                if (itSrc.second->timer != NULL)
                { // Debounce is existing
                    if (!itSrc.second->isTimerActive)
                    { // Debounce timer is not active - start it!
                        itSrc.second->isTimerActive = true;
                        xTimerStart(itSrc.second->timer, 0);
                    }
                    else
                    {
                        // Debounce timer is active - skip it
                    }
                }
                else
                {
                    // No debounce
                    itSrc.second->value = val;
                    esp_event_post_to(GetEventLoop(), GetEventBase(), SIGMAIO_EVENT_PIN, itSrc.second, sizeof(PinValue), portMAX_DELAY);
                }
            }
        }
    }
}
