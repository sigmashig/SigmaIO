#include "SigmaIO.h"
#include "SigmaGPIO.h"
#include "SigmaPCF8575.h"
#include "SigmaPCA9685.h"

void SigmaIO::init()
{
    isInit = true;
    //Serial.println("SigmaIO init");
    RegisterPinDriver(SIGMAIO_GPIO, NULL, 0, GPIO_PIN_COUNT - 1);
    if (eventLoop == NULL)
    {
        esp_event_loop_args_t loop_args = {
            .queue_size = (int32_t)100,
            .task_name = "SigmaIO",
            .task_priority = 50,
            .task_stack_size = 4096,
            .task_core_id = 0};

        esp_event_loop_create(&loop_args, (void**)&eventLoop);
        if (eventLoop == NULL)
        {
            Serial.println("SigmaIO: event loop creation failed");
        }
    }
    esp_event_handler_register_with(eventLoop, eventBase, SIGMAIO_EVENT_DIRTY, processInterrupt, NULL);
}
/*
SigmaIO::~SigmaIO()
{
    esp_event_handler_unregister(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, &processInterrupt);
    for (auto it : pinRangeDriverSet)
    {
        if (it.second.isInternal)
        {
            delete it.second.pinDriver;
        }
    }
    pinRangeDriverSet.clear();
    pinDriverSet.clear();

    for (auto it : interruptMap)
    {
        DetachInterruptAll(it.first);
    }
    interruptMap.clear();
}
*/

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
        // pin is not initialized yet
        for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
        {
            pdd = it->second;

            if (it->first <= pin)
            {
                if (pin <= pdd.end)
                { // range found
                    std::pair<int, PinDriverDefinition> newPair = {pin, pdd};
                    pinDriverSet.insert(newPair);

                    res = SIGMAIO_SUCCESS;
                    break;
                }
            }
        }
    }
    if (res == SIGMAIO_SUCCESS)
    {
        pdd.pinDriver->PinMode(pin - pdd.beg, mode);
    }
    return res;
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

    //Serial.printf("checkDriverRegistrationAbility: pinBegin: %d, pinEnd: %d\n", pinBegin, pinEnd);
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

IOError SigmaIO::RegisterPinDriver(SigmaIoDriver driverCode, void *drvParams, uint pinBegin, uint numberPins)
{
    if (!isInit)
    {
        init();
    }
    IOError res = checkDriverRegistrationAbility(pinBegin, numberPins);
    if (res != SIGMAIO_SUCCESS)
    {
        return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
    }
    SigmaAbstractPinDriver *pinDriver = nullptr;
    switch (driverCode)
    {
    case SIGMAIO_GPIO:
    {
        pinDriver = new SigmaGPIO();
        break;
    }
    case SIGMAIO_PCF8575:
    {
        I2CParams *i2cParams = (I2CParams *)drvParams;
        if (i2cParams == nullptr)
        {
            return SIGMAIO_ERROR_BAD_DRIVER_PARAMS;
        }
        if (i2cParams->pWire == nullptr)
        {
            pinDriver = new SigmaPCF8575IO(i2cParams->address);
        }
        else
        {
            pinDriver = new SigmaPCF8575IO(i2cParams->address, i2cParams->pWire, i2cParams->sda, i2cParams->scl);
        }
        break;
    }
    case SIGMAIO_PCA9685:
    {
        I2CParams *i2cParams = (I2CParams *)drvParams;
        if (i2cParams == nullptr)
        {
            return SIGMAIO_ERROR_BAD_DRIVER_PARAMS;
        }
        pinDriver = new SigmaPCA9685IO(i2cParams->address, 0, i2cParams->pWire);
        break;
    }
    default:
        return SIGMAIO_ERROR_BAD_DRIVER_CODE;
    }
    if (numberPins == 0)
    {
        numberPins = pinDriver->GetNumberOfPins();
    }
    uint pinEnd = pinBegin + numberPins - 1;
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, true, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    pinDriver->AfterRegistration(newPair.second);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, uint pinBegin, uint numberPins)
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
    //Serial.printf("RegisterPinDriver: pinBegin: %d, numberPins: %d\n", pinBegin, numberPins);
    IOError res = checkDriverRegistrationAbility(pinBegin, numberPins);
    if (res != SIGMAIO_SUCCESS)
    {
        return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
    }
    uint pinEnd = pinBegin + numberPins - 1;
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, false, pinDriver}};
    //Serial.printf("RegisterPinDriver: newPair: %d, %d-%d\n", newPair.first, newPair.second.beg, newPair.second.end);
    pinRangeDriverSet.insert(newPair);
    //Serial.printf("RegisterPinDriver: pinRangeDriverSet.size(): %d\n", pinRangeDriverSet.size());
    pinDriver->AfterRegistration(newPair.second);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver)
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

IOError SigmaIO::RegisterPwmPin(uint pin, uint frequency, byte resolution, uint minValue, uint maxValue)
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
            if (pdd.pinDriver->RegisterPwmPin(pin - pdd.beg, frequency, resolution, minValue, maxValue))
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
    //Serial.printf("SetPwm: pin: %d, value: %d\n", pin, value);
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        //Serial.printf("SetPwm: pinDriver: %s\n", pdd.pinDriver->GetPinDriverName().c_str());
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
    //Serial.printf("GetPinDriver: pin: %d\n", pin);
    if (pinDriverSet.size() != 0)
    {
        //Serial.printf("GetPinDriver: pinDriverSet.size(): %d\n", pinDriverSet.size());
        auto it = pinDriverSet.find(pin);
        if (it != pinDriverSet.end())
        {
            //Serial.printf("GetPinDriver: pinDriver: %s\n", it->second.pinDriver->GetPinDriverName().c_str());
            return it->second;
        }
    }
    return {0, 0, false, nullptr};
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
