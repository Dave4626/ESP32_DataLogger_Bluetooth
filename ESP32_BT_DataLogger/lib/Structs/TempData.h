#ifndef TempData_h
#define TempData_h

#include <Arduino.h>

typedef struct
{
    uint16_t Temperature;
    uint16_t Humidity;
    time_t Time;
} TempData, *pTempData;

typedef union {
   TempData tempData; 
   byte bytes[8];
} TempDataUnion;

String GetTempDataAsString(pTempData data);

bool IsEmpty(pTempData);

#endif