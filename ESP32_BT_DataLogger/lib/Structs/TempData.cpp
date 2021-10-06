#include <Arduino.h>
#include "TempData.h"

String GetTempDataAsString(pTempData data)
{
    return "Temperature: " + String(data->Temperature) +
           "C Humidity: " + String(data->Humidity) +
           "% Hours from compile: " + String(data->HoursFromCompile);
}

TempData FromUint32(uint32_t i)
{
    uint8_t temp = (uint8_t)(i >> 24);
    uint8_t humi = (uint8_t)(i >> 16);
    uint16_t hours = (uint16_t)(i & 0xFFFF);
    TempData t = {Temperature : temp, Humidity : humi, HoursFromCompile : hours};

    return t;
}

uint32_t ToUint32(pTempData dataPtr)
{
    return (uint32_t)((dataPtr->Temperature << 24) | (dataPtr->Humidity << 16) | dataPtr->HoursFromCompile);
}