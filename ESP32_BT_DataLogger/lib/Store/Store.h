#ifndef Store_h
#define Store_h

#include <Arduino.h>
#include <EEPROM.h>
#include "TempData.h"

class Store
{
public:
    TempData getValueFromEEPROM(uint16_t index);
    void setValueToEEPROM(pTempData dataPtr, uint16_t index);
    void setValueToEEPROM(pTempData dataPtr);
    void clearEEPROM();
    uint16_t getMaximalIndex();

private:
    bool isInitialized = false;
    void init();
};

#endif