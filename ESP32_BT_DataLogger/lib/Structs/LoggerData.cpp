#include <Arduino.h>
#include "LoggerData.h"
#include <RTClib.h>

String GetLoggerDataAsString(pLoggerData data)
{
    DateTime d = DateTime(data->Time);
    char buffer[21];
    sprintf(buffer, "%02d.%02d.%04d %02d:%02d:%02d", d.day(), d.month(), d.year(), d.hour(), d.minute(), d.second());
    
    return "Temperature;" + String(data->Temperature / 10.0) +
           ";[C];Humidity;" + String(data->Humidity / 10.0) +
           ";[%];Date;" + String(buffer);
}

bool IsEmpty(pLoggerData data){
    return data->Temperature == 0 && data->Humidity == 0 && data->Time == 0;
}

