#include <Arduino.h>
#include <EEPROM.h>
#include "Store.h"
#include "TempData.h"

const uint16_t EEPROMSize = 512;
const uint8_t blockSize = 8;

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

    if (index > (EEPROMSize / blockSize))
    {
        return {0, 0, 0};
    }

    //index x 4bytes because each value has 4bytes
    TempData t = TempData();
    EEPROM.readBytes(index * blockSize, &t, blockSize);
    Serial.println("EEPROM read at index: " + String(index) + " has value: " + GetTempDataAsString(&t));

    return t;
}

void Store::setValueToEEPROM(pTempData dataPtr, uint16_t index)
{
    if (isInitialized == false)
        init();

    if (index > (EEPROMSize / blockSize))
    {
        Serial.println("index is out of range");
        return;
    }

    EEPROM.writeBytes(index * blockSize, dataPtr, blockSize);
    EEPROM.commit();

    Serial.println("Saved value to EEPROM: " + GetTempDataAsString(dataPtr) + " at index " + String(index));
}

void Store::setValueToEEPROM(pTempData dataPtr)
{
    if (isInitialized == false)
        init();

    //Get the last index
    uint16_t index = 0;
    bool foundIndex = false;
    while (index < (EEPROMSize / blockSize))
    {
        byte bytes[blockSize];
        EEPROM.readBytes(index * blockSize, &bytes, blockSize);
        bool allZero = true;
        for (size_t i = 0; i < blockSize; i++)
        {
            if (bytes[i] != 0)
            {
                allZero = false;
                break;
            }
        }

        if (allZero == true)
        {
            //first empty value (at index) can be used for write
            foundIndex = true;
            break;
        }
        index++;
    }

    if (!foundIndex)
    {
        Serial.println("Canot find last empty index - EEPROM is full");
        return;
    }

    setValueToEEPROM(dataPtr, index);
}

uint16_t Store::getMaximalIndex()
{
    return EEPROMSize / blockSize;
}
