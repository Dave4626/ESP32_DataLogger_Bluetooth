#ifndef LoggerConfig_h
#define LoggerConfig_h

#include <Arduino.h>

typedef struct
{
    uint32_t period;
    uint32_t _reserved;
} LoggerConfig, *pLoggerConfig;

typedef union {
   LoggerConfig loggerConfig; 
   byte bytes[8];
} LoggerConfigUnion;

#endif