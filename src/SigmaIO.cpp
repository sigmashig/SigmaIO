#include "SigmaIO.hpp"
#include "SigmaGPIO.hpp"

SigmaIO::SigmaIO()
{
    gpio = new SigmaGPIO();
    RegisterPinDriver(gpio, 0, GPIO_PIN_COUNT - 1);
    esp_event_handler_register(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, processInterrupt, NULL);
}

SigmaIO::~SigmaIO()
{
    esp_event_handler_unregister(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, &processInterrupt);
    pinRangeDriverSet.clear();
    pinDriverSet.clear();

    for (auto it : interruptMap)
    {
        DetachInterruptAll(it.first);
    }
    interruptMap.clear();
    delete gpio;
}

IOError SigmaIO::DetachInterruptAll(uint pinIsr)
{
    for (auto it : interruptMap)
    {
        if (it.first == pinIsr)
        {
            for (auto it2 : it.second.pinSrcMap)
            {
                DetachInterrupt(pinIsr, it2.first);
            }
        }
    }
    return SIGMAIO_SUCCESS;
}

IOError SigmaIO::PinMode(uint pin, byte mode)
{
    PinDriverDefinition pdd = getPinDriver(pin);
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
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        pdd.pinDriver->DigitalWrite(pin - pdd.beg, value);
    }
}

byte SigmaIO::DigitalRead(uint pin)
{
    PinDriverDefinition pdd = getPinDriver(pin);
    if (pdd.pinDriver != nullptr)
    {
        return pdd.pinDriver->DigitalRead(pin - pdd.beg);
    }
    return 0xFF;
}

IOError SigmaIO::RegisterPinDriver(SigmaAbstractPinDriver *pinDriver, byte pinBegin, byte pinEnd)
{
    // PinDriverRange range = {pinBegin, pinEnd};
    if (pinBegin > pinEnd)
    {
        return SIGMAIO_ERROR_BAD_PIN_RANGE;
    }
    if (pinDriver == NULL)
    {
        return SIGMAIO_ERROR_BAD_PIN_DRIVER;
    }
    for (auto it = pinRangeDriverSet.begin(); it != pinRangeDriverSet.end(); it++)
    {
        PinDriverDefinition pdd = it->second;
        if (pdd.pinDriver == pinDriver)
        {
            if (pdd.beg == pinBegin && pdd.end == pinEnd)
            {
                return SIGMAIO_SUCCESS;
            }
            else
            {
                return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
            }
        }
        if (it->first > pinBegin)
        { // potentially found a hole. check if it is not overlapping with end
            if (it->first <= pinEnd)
            {
                return SIGMAIO_ERROR_PIN_RANGE_ALREADY_REGISTERED;
            }
            else
            {
                std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, pinDriver}};
                pinRangeDriverSet.insert(newPair);
                return SIGMAIO_SUCCESS;
            }
        }
    }
    std::pair<int, PinDriverDefinition> newPair = {pinBegin, {pinBegin, pinEnd, pinDriver}};
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
    PinDriverDefinition pdd = getPinDriver(pin);
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
    PinDriverDefinition pdd = getPinDriver(pin);
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

IOError SigmaIO::AttachInterrupt(uint pinIsr, uint pinSrc, uint debounceTime, int mode)
{
    Serial.printf("AttachInterrupt %d %d %d %d\n", pinIsr, pinSrc, debounceTime, mode);

    InterruptDescription isrDescr;
    auto it = interruptMap.find(pinIsr);
    if (it == interruptMap.end())
    {
        // New PinIsr
        bool val = DigitalRead(pinSrc);
        std::pair<uint, bool> newPairSrc = {pinSrc, val};
        isrDescr.pinIsr = pinIsr;
        isrDescr.debounceTime = debounceTime;
        isrDescr.isEnabled = true;
        isrDescr.pinSrcMap.insert(newPairSrc);

        std::pair<uint, InterruptDescription> newPair = {pinIsr,isrDescr};
        interruptMap.insert(newPair);
        auto x = interruptMap.find(pinIsr);
        attachInterruptArg(pinIsr, processISR, &(x->second), mode);

        return SIGMAIO_SUCCESS;
    }
    else
    {
        auto src = it->second.pinSrcMap.find(pinSrc);
        if (src != it->second.pinSrcMap.end())
        {
            // PinSrc already attached
            return SIGMAIO_ERROR_INTERRUPT_ALREADY_ATTACHED;
        }
        else
        {
            bool val = DigitalRead(pinSrc);
            std::pair<uint, bool> newPair = {pinSrc, val};
            it->second.pinSrcMap.insert(newPair);
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
        auto src = it->second.pinSrcMap.find(pinSrc);
        if (src != it->second.pinSrcMap.end())
        {
            // PinSrc attached
            it->second.pinSrcMap.erase(src);
            if (it->second.pinSrcMap.size() == 0)
            {
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

SigmaIO::PinDriverDefinition SigmaIO::getPinDriver(uint pin)
{
    if (pinDriverSet.size() != 0)
    {
        auto it = pinDriverSet.find(pin);
        if (it != pinDriverSet.end())
        {
            return it->second;
        }
    }
    return {0, 0, nullptr};
}

ICACHE_RAM_ATTR void SigmaIO::processISR(void *arg)
{
    sigmaIO->isrCnt++;
    InterruptDescription *descr = (InterruptDescription *)arg;
    sigmaIO->p = descr->pinIsr;
    if (descr->isEnabled)
    {
        if (descr->debounceTime != 0)
        {
            descr->isEnabled = false;
        }
        sigmaIO->err = esp_event_isr_post(SIGMAIO_EVENT, SIGMAIO_EVENT_DIRTY, &(descr->pinIsr), sizeof(descr->pinIsr), NULL);
    }
}

void SigmaIO::processInterrupt(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    byte *pin = (byte *)event_data;
    Serial.println("processInterrupt");
    Serial.printf("P=%d rin=%d\n", sigmaIO->p, *pin);
    return;
    /*
    InterruptKey *key = (InterruptKey *)event_data;
    auto it = interruptMap.find(*key);
    if (it != interruptMap.end())
    {
        for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++)
        {
            PinValue id = it1->second;
            bool val = sigmaIO->DigitalRead(it1->first);
            if (id.value != val)
            {
                if (id.debounceTime == 0)
                {
                    id.value = val;
                    esp_event_isr_post(SIGMAIO_EVENT, SIGMAIO_EVENT_PIN, &id, sizeof(PinValue), NULL);
                }
                else
                {
                    TimerHandle_t xTimer = xTimerCreate("debounce", pdMS_TO_TICKS(id.debounceTime), pdFALSE, (void *)&it1->second,
                                                        [](TimerHandle_t xTimer)
                                                        {
                                                            PinValue *id = (PinValue *)pvTimerGetTimerID(xTimer);
                                                            byte val = sigmaIO->DigitalRead(id->pinSrc);
                                                            if (id->value != val)
                                                            {
                                                                id->value = val;
                                                                esp_event_isr_post(SIGMAIO_EVENT, SIGMAIO_EVENT_PIN, id, sizeof(PinValue), NULL);
                                                            }
                                                            xTimerDelete(xTimer, 0);
                                                        });
                    xTimerStart(xTimer, 0);
                }
            }
        }
    }*/
}

//-----------------------------------------------------
std::map<uint, InterruptDescription> SigmaIO::interruptMap;
SigmaIO *sigmaIO;
ESP_EVENT_DEFINE_BASE(SIGMAIO_EVENT);
