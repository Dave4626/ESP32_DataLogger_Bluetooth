#ifndef LoggerConfig_h
#define LoggerConfig_h

#include <Arduino.h>

typedef struct
{
    uint8_t startHour;
    uint8_t startMinute;
    uint16_t periodMinutes;
    uint32_t _reserved;
} LoggerConfig, *pLoggerConfig;

typedef union {
   LoggerConfig loggerConfig; 
   byte bytes[8];
} LoggerConfigUnion;

#endif