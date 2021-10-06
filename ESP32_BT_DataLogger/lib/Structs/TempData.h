#ifndef TempData_h
#define TempData_h

#include <Arduino.h>

typedef struct
{
    uint8_t Temperature;
    uint8_t Humidity;
    uint16_t HoursFromCompile;
} TempData, *pTempData;

String GetTempDataAsString(pTempData data);

TempData FromUint32(uint32_t i);

uint32_t ToUint32(pTempData i);

#endif