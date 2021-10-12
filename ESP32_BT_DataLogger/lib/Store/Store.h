#ifndef Store_h
#define Store_h

#include <Arduino.h>
#include <EEPROM.h>
#include "LoggerData.h"
#include "LoggerConfig.h"

class Store
{
public:
    LoggerData getValueFromEEPROM(uint16_t index);
    void setValueToEEPROM(pLoggerData dataPtr, uint16_t index);
    bool setValueToEEPROM(pLoggerData dataPtr);
    void clearEEPROM();
    uint16_t getMaximalIndex();
    LoggerConfig getConfig();
    void setLoggerConfig(pLoggerConfig configPtr);

private:
    bool isInitialized = false;
    void init();
    void writeEEPROM(unsigned int eeaddress, byte data);
    byte readEEPROM(unsigned int eeaddress);
    bool isEEPROMReady();
};

#endif