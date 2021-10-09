#include <Arduino.h>
#include "TempData.h"
#include <RTClib.h>

String GetTempDataAsString(pTempData data)
{
    DateTime d = DateTime(data->Time);
    String date =
        String(d.day()) + "." +
        String(d.month()) + "." +
        String(d.year()) + " " +
        String(d.hour()) + ":" +
        String(d.minute()) + ":" +
        String(d.second());

    return "Temperature: " + String(data->Temperature) +
           "°C Humidity: " + String(data->Humidity) +
           "% Date: " + String(date);
}

bool IsEmpty(pTempData data){
    return data->Temperature == 0 && data->Humidity == 0 && data->Time == 0;
}

