#include <Arduino.h>
#include <Wire.h>
#include "Store.h"
#include "LoggerData.h"

const int eepromAddress = 0x50;
const uint16_t EEPROMSize = 4096;
const uint8_t blockSize = 8;
//configuration takes 1 block (at last index)
const uint8_t configBlocksCount = 1;

bool Store::isEEPROMReady(){
    if (isInitialized == false)
        init();
    Wire.beginTransmission(eepromAddress);
    return !Wire.endTransmission();
}

//Write 1byte at specified address
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

//Read 1byte at specified address
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

//Init EEPROM
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

//Clear all data, (not configuration)
void Store::clearEEPROM()
{
    if (isInitialized == false)
        init();

    //clear data (not configuration)
    for (size_t i = 0; i < EEPROMSize - (blockSize * configBlocksCount); i++)
    {
        writeEEPROM(i, (byte)0);
    }
    Serial.println("EEPROM cleared to 0");
}

//Get value at specified index
LoggerData Store::getValueFromEEPROM(uint16_t index)
{
    if (isInitialized == false)
        init();

    if (index + configBlocksCount > (EEPROMSize / blockSize))
    {
        return {0, 0, 0};
    }

    LoggerDataUnion *tu = new LoggerDataUnion();
    for (size_t i = 0; i < blockSize; i++)
    {
        tu->bytes[i] = readEEPROM((index * blockSize) + i);
    }
    //Serial.println("EEPROM read at index: " + String(index) + " has value: " + GetLoggerDataAsString(&(tu->loggerData)));

    return tu->loggerData;
}

//Set value at specified index
void Store::setValueToEEPROM(pLoggerData dataPtr, uint16_t index)
{
    if (isInitialized == false)
        init();

    if (index > ((EEPROMSize - (blockSize * configBlocksCount)) / blockSize))
    {
        Serial.println("index is out of range");
        return;
    }

    LoggerDataUnion tu;
    tu.loggerData = *dataPtr;
    for (size_t i = 0; i < blockSize; i++)
    {
        writeEEPROM(index * blockSize + i, tu.bytes[i]);
    }

    Serial.println("Saved value to EEPROM: " + GetLoggerDataAsString(dataPtr) + " at index " + String(index));
}

//Set value at lest empty index
bool Store::setValueToEEPROM(pLoggerData dataPtr)
{
    if (isInitialized == false)
        init();

    //Get the last index
    uint16_t index = 0;
    bool foundIndex = false;
    while (index < ((EEPROMSize - (blockSize * configBlocksCount)) / blockSize))
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
        return false;
    }

    setValueToEEPROM(dataPtr, index);
    return true;
}

//Get maximal index
uint16_t Store::getMaximalIndex()
{
    return (EEPROMSize - (blockSize * configBlocksCount)) / blockSize;
}

//Get config
LoggerConfig Store::getConfig(){
    uint32_t configIndex = EEPROMSize - (configBlocksCount * blockSize);

    LoggerConfigUnion *cu = new LoggerConfigUnion();
    for (size_t i = 0; i < (configBlocksCount * blockSize); i++)
    {
        cu->bytes[i] = readEEPROM(configIndex + i);
    }

    Serial.println("EEPROM read at index: " + String(configIndex));

    return cu->loggerConfig;
}

//Save config
void Store::setLoggerConfig(pLoggerConfig configPtr){
    uint32_t configIndex = EEPROMSize - (configBlocksCount * blockSize);

    LoggerConfigUnion cu;
    cu.loggerConfig = *configPtr;
    for (size_t i = 0; i < (configBlocksCount * blockSize); i++)
    {
        writeEEPROM(configIndex + i, cu.bytes[i]);
    }

    Serial.println("Saved value to EEPROM: " + String(configPtr->period) + " at index " + String(configIndex));
}