#include <Arduino.h>
#include <EEPROM.h>
#include "Store.h"
#include "TempData.h"

const uint16_t EEPROMSize = 512;

void Store::init()
{
    EEPROM.begin(EEPROMSize);
    isInitialized = true;
    Serial.println("EEPROM is initialized");
}

void Store::clearEEPROM()
{
    if (isInitialized == false)
        init();

    for (size_t i = 0; i < EEPROMSize; i++)
    {
        EEPROM.writeByte(i, 0);
    }
    EEPROM.commit();
    Serial.println("EEPROM cleared to 0");
}

TempData Store::getValueFromEEPROM(uint16_t index)
{
    if (isInitialized == false)
        init();

    if (index > (EEPROMSize / 4))
    {
        return {0,0,0};
    }

    //index x 4bytes because each value has 4bytes
    uint32_t value = EEPROM.readUInt(index * 4);
    Serial.println("EEPROM value: " + String(value));
    TempData t = FromUint32(value);

    return t;
}

void Store::setValueToEEPROM(pTempData dataPtr, uint16_t index)
{
    if (isInitialized == false)
        init();
    
    if (index > (EEPROMSize / 4))
    {
        Serial.println("index is out of range");
        return;
    }

    uint32_t value = ToUint32(dataPtr);
    uint32_t currentValue = EEPROM.readUInt(index * 4);
    if (currentValue != value)
    {
        //save only when different (save write cycles)
        EEPROM.writeUInt(index * 4, value);
        EEPROM.commit();
    }
    Serial.println("Saved value to EEPROM: " + String(value) + " at index " + String(index));
}

void Store::setValueToEEPROM(pTempData dataPtr)
{
    if (isInitialized == false)
        init();

    //Get the last index
    uint16_t index = 0;
    bool foundIndex = false;
    while (index < (EEPROMSize / 4)){
        uint32_t someValue = EEPROM.readUInt(index * 4);
        if (someValue == 0){
            //first empty value (at index) can be used for write
            foundIndex = true;
            break;
        }
        index++;
    }

    if (!foundIndex || index > (EEPROMSize / 4))
    {
        Serial.println("EEPROM is full");
        return;
    }

    uint32_t value = ToUint32(dataPtr);

    uint32_t currentValue = EEPROM.readUInt(index * 4);
    if (currentValue != value)
    {
        //save only when different (save write cycles)
        EEPROM.writeUInt(index * 4, value);
        EEPROM.commit();
    }
    Serial.println("Saved value to EEPROM: " + GetTempDataAsString(dataPtr) + " at index " + String(index));
}

uint16_t Store::getMaximalIndex(){
    return EEPROMSize / 4;
}
