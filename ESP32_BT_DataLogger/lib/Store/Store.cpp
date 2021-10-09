#include <Arduino.h>
#include <Wire.h>
#include "Store.h"
#include "TempData.h"

const int eepromAddress = 0x50;
const uint16_t EEPROMSize = 4096;
const uint8_t blockSize = 8;

bool Store::isEEPROMReady(){
    if (isInitialized == false)
        init();
    Wire.beginTransmission(eepromAddress);
    return !Wire.endTransmission();
}

void Store::writeEEPROM(unsigned int eeaddress, byte data)
{
    if (isInitialized == false)
        init();

    while(!isEEPROMReady()){
        delay(10);
    }

    Wire.beginTransmission(eepromAddress);
    Wire.write((int)(eeaddress >> 8));   //writes the MSB
    Wire.write((int)(eeaddress & 0xFF)); //writes the LSB
    Wire.write(data);
    if (Wire.endTransmission() != I2C_ERROR_OK){
        Serial.println("Byte NOT written to EEPROM");
    }
}

byte Store::readEEPROM(unsigned int eeaddress)
{
    if (isInitialized == false)
        init();
    
    while(!isEEPROMReady()){
        delay(10);
    }

    byte rdata = 0xFF;
    Wire.beginTransmission(eepromAddress);
    Wire.write((int)(eeaddress >> 8));   //writes the MSB
    Wire.write((int)(eeaddress & 0xFF)); //writes the LSB
    if (Wire.endTransmission() != I2C_ERROR_OK){
        Serial.println("Read from EEPROM Error (1st)");
    }
    Wire.requestFrom(eepromAddress, 1);

    if (Wire.available()){
        rdata = Wire.read();
        return rdata;
    }
    else{
        Serial.println("EEPROM read error (2nd)");
        return (byte)0x00;
    }
}

void Store::init()
{
    if (!Wire.begin())
    {
        Serial.println("EEPROM is NOT initialized");
    }
    else
    {
        isInitialized = true;
        Serial.println("EEPROM is initialized");
    }
}

void Store::clearEEPROM()
{
    if (isInitialized == false)
        init();

    for (size_t i = 0; i < EEPROMSize; i++)
    {
        writeEEPROM(i, (byte)0);
    }
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
    TempDataUnion *tu = new TempDataUnion();
    for (size_t i = 0; i < blockSize; i++)
    {
        tu->bytes[i] = readEEPROM((index * blockSize) + i);
    }
    //Serial.println("EEPROM read at index: " + String(index) + " has value: " + GetTempDataAsString(&(tu->tempData)));

    return tu->tempData;
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

    TempDataUnion tu;
    tu.tempData = *dataPtr;
    for (size_t i = 0; i < blockSize; i++)
    {
        writeEEPROM(index * blockSize + i, tu.bytes[i]);
    }

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
        for (size_t i = 0; i < blockSize; i++)
        {
            bytes[i] = readEEPROM((index * blockSize) + i);
        }
        
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
