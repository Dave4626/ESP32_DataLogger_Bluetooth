# ESP32 DataLogger with Bluetooth
This project is simple data logger (periodiccaly log temperature and humidity to ext EEPROM). Logger is also ready to connect to your mobile phone via Bluetooth and print the stored data/or be configured etc.

## Logical Diagram
![esp data logger diagram](/media/DataLogger-dark.png)

## Logging
In this project I will read temperature and humidity using a simple DHT11 sensor and log it to ext EEPROM. The measurement period will be in the order of hours, so not often. In the meantime, ESP will be in deep sleep mode. 

## Data storage
The data will be stored in external EEPROM. To save the last write address, I do not use some kind of index (to limit the number of writes per one location). At the beginning, I delete the EEPROM (via BT) and then log writes to the first non-zero place. 

## Data structure
Data structure is simple 8 bytes block, where temperature and humidity is stored as integer uint16 (read temperature x 10 (eg temperature is 31.5 => store 315)) And Time is standard unix time (number of sec from 1970).

    uint16_t Temperature;
    uint16_t Humidity;
    time_t Time;

## Two modes - interrupt from button or from RTC
In the standard state, ESP is in deep sleep mode. 
- Interrupt may come from the RTC (At that moment, it measures the temperature and writes it to the log)
- Or an interrupt may occur from pressing a button, in which case it switches to Bluetooth mode and waits for a mobile connection. 

## Phone connection
If Esp is in bluetooth mode, it is waiting for the phone to connect. Once connected, it will print a welcome message with list of possible commands. And he awaits instructions.
Instructions can be:
- **read** for start reading stored data
- **clear** for clear the EEPROM
- etc...

It can be also configured via BT (config is stored in EEPROM same as data). As soon as it receives the "read" instruction, it starts sending measured values via BT. 
