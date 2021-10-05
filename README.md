# ESP32 DataLogger with Bluetooth
This project is simple data logger, which is ready to connect to mobile phone via Bluetooth and print the stored data.

## Logging
In this project I will log temperature and humidity using a simple DHT11 sensor. The measurement period will be in the order of hours, so not often.

## Data storage
The data will be stored in the internal EEPROM. To save the last write address, I do not use some kind of index. At the beginning, I delete the EEPROM and then write to the first non-zero place. It means that I can never write zero as a valid value, and it is necessary to handle it in the code.

## Two modes - interrupt button
By default, Esp works in logger mode, but if the button is pressed, it switches to bluetooth mode.

## Phone connection
If Esp is in bluetooth mode, it is waiting for the phone to connect. Once connected, it will print a welcome message. And he awaits instructions.
Instructions can be:
- **start** for start reading stored data
- **clear** for clear the EEPROM
- **end** for exit bluetooth mode
As soon as it receives the "start" instruction, it starts sending measured values via BT. 
