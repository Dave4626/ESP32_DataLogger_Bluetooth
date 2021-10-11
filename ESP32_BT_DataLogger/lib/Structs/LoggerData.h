#ifndef LoggerData_h
#define LoggerData_h

#include <Arduino.h>

typedef struct
{
    uint16_t Temperature;
    uint16_t Humidity;
    time_t Time;
} LoggerData, *pLoggerData;

typedef union {
   LoggerData loggerData; 
   byte bytes[8];
} LoggerDataUnion;

String GetLoggerDataAsString(pLoggerData data);

bool IsEmpty(pLoggerData);

#endif