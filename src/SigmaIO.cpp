#include "SigmaIO.h"
#include "SigmaGPIO.h"
#include "SigmaPCF8575.h"

SigmaIO::SigmaIO(bool isRegisterGPIO)
{
    if (isRegisterGPIO)
    {
        RegisterPinDriver(SIGMAIO_GPIO, NULL, 0, GPIO_PIN_COUNT - 1);
    }
    esp_event_handler_register(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, processInterrupt, NULL);
}

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

IOError SigmaIO::PinMode(uint pin, byte mode)
{
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

void SigmaIO::DigitalWrite(uint pin, byte value)
{
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        pdd.pinDriver->DigitalWrite(pin - pdd.beg, value);
    }
}

byte SigmaIO::DigitalRead(uint pin)
{
    PinDriverDefinition pdd = GetPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        return pdd.pinDriver->DigitalRead(pin - pdd.beg);
    }
    return 0xFF;
}

IOError SigmaIO::checkDriverRegistrationAbility(byte pinBegin, byte pinEnd)
{
    // PinDriverRange range = {pinBegin, pinEnd};
    if (pinBegin > pinEnd)
    {
        return SIGMAIO_ERROR_BAD_PIN_RANGE;
    }
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

IOError SigmaIO::RegisterPinDriver(SigmaIoDriver driverCode, void *drvParams, byte pinBegin, byte pinEnd)
{
    IOError res = checkDriverRegistrationAbility(pinBegin, pinEnd);
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
            pinDriver = new SigmaPCF8575IO(i2cParams->pWire, i2cParams->address, i2cParams->sda, i2cParams->scl);
        }
        break;
    }
    default:
        return SIGMAIO_ERROR_BAD_DRIVER_CODE;
    }
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, true, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd)
{
    if (pinDriver == NULL)
    {
        return SIGMAIO_ERROR_BAD_PIN_DRIVER;
    }
    IOError res = checkDriverRegistrationAbility(pinBegin, pinEnd);
    if (res != SIGMAIO_SUCCESS)
    {
        return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
    }

    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, false, pinDriver}};
    pinRangeDriverSet.insert(newPair);
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::UnregisterPinDriver(SigmaAbstractPinDriver *pinDriver)
{
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
    for (auto isrIt : sigmaIO->interruptMap)
    {
        for (auto srcIt : isrIt.second->pinSrcMap)
        {
            if (srcIt.second->timer == xTimer)
            { // Timer found
                bool val = sigmaIO->DigitalRead(srcIt.first);
                //Serial.printf("checkDebounced: pin: %d, OLD value: %d, NEW value: %d\n", srcIt.first, srcIt.second->value, val);
                srcIt.second->isTimerActive = false;
                if (srcIt.second->value != val)
                { // The real input value has changed
                    srcIt.second->value = val;
                    esp_event_post(SIGMAIO_EVENT, SIGMAIO_EVENT_PIN1, srcIt.second, sizeof(PinValue), portMAX_DELAY);
                }
            }
        }
    }
}

IOError SigmaIO::AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime, int mode)
{
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
    if (pinDriverSet.size() != 0)
    {
        auto it = pinDriverSet.find(pin);
        if (it != pinDriverSet.end())
        {
            return it->second;
        }
    }
    return {0, 0, false, nullptr};
}

ICACHE_RAM_ATTR void SigmaIO::processISR(void *arg)
{
    InterruptDescription *descr = (InterruptDescription *)arg;
    esp_event_isr_post(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, &(descr->pinIsr), sizeof(descr->pinIsr), NULL);
}

void SigmaIO::processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint *pin = (uint *)event_data;
    auto itIsr = interruptMap.find(*pin);
    //Serial.printf("processInterrupt: pin: %d\n", *pin);
    if (itIsr != interruptMap.end())
    {
        for (auto itSrc : itIsr->second->pinSrcMap)
        {
            bool val = sigmaIO->DigitalRead(itSrc.first);
            if (itSrc.second->value != val)
            {
               // Serial.printf("processInterrupt: pin: %d, OLD value: %d, NEW value: %d\n", itSrc.first, itSrc.second->value, val);
                if (itSrc.second->timer != NULL)
                { // Debounce is existing
                    // if (xTimerIsTimerActive(itSrc.second->timer) == pdFALSE)
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
                    esp_event_post(SIGMAIO_EVENT, SIGMAIO_EVENT_PIN, itSrc.second, sizeof(PinValue), portMAX_DELAY);
                }
            }
        }
    }
}

void SigmaIO::Begin()
{
    for (auto it : pinRangeDriverSet)
    {
        it.second.pinDriver->Begin();
    }
}

//-----------------------------------------------------
std::map<uint, InterruptDescription *> SigmaIO::interruptMap;
SigmaIO *sigmaIO;
ESP_EVENT_DEFINE_BASE(SIGMAIO_EVENT);
