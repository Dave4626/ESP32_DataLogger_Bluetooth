#ifndef Store_h
#define Store_h

#include <Arduino.h>
#include <EEPROM.h>

class Store
{
public:
    uint32_t getValueFromEEPROM(uint16_t index);
    void setValueToEEPROM(uint32_t value, uint16_t index);
    void setValueToEEPROM(uint32_t value);
    void clearEEPROM();
    uint16_t getMaximalIndex();

private:
    bool isInitialized = false;
    void init();
};

#endif