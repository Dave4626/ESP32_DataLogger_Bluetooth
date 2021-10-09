#ifndef TempData_h
#define TempData_h

#include <Arduino.h>

typedef struct
{
    uint16_t Temperature;
    uint16_t Humidity;
    time_t Time;
} TempData, *pTempData;

String GetTempDataAsString(pTempData data);

bool IsEmpty(pTempData);

#endif