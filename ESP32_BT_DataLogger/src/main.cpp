#include <Arduino.h>
#include "BluetoothSerial.h"
#include "Store.h"

//Define constants
#define BUTTONPIN 0
#define LEDPIN 4

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Global variables
volatile bool bluetoothSwitch = false;
static unsigned long last_interrupt_time = 0;
BluetoothSerial SerialBT;
Store EEPROMStore;

//debounce for button interrupt
bool debounce()
{
  bool result = false;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200)
  {
    result = true;
  }
  last_interrupt_time = interrupt_time;

  return result;
}

//switch state
void stateSwitch(bool state)
{
  bluetoothSwitch = state;
  digitalWrite(LEDPIN, state ? HIGH : LOW);
}

//interrupt handler
void buttonPressed()
{
  if (debounce())
  {
    Serial.println("BUTTON PRESSED");
    if (bluetoothSwitch == true)
    {
      stateSwitch(false);
    }
    else
    {
      stateSwitch(true);
    }
  }
}

//Setup
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("START");
  pinMode(LEDPIN, OUTPUT);
  attachInterrupt(BUTTONPIN, buttonPressed, RISING);
}

//Program running when bluetooth flag is off
void programWhenBTOff()
{
  Serial.println("Led is now OFF " + String(millis()));
  uint32_t timeInSeconds = millis() / 1000;
  EEPROMStore.setValueToEEPROM(timeInSeconds);
  delay(2000);
}

//Program running when BT is ON
void programWhenBTOn()
{
  SerialBT.begin("ESP32_DATALOGGER_BT");
  Serial.println("The device started, now you can pair it with bluetooth!");

  bool sentMessage = false;
  while (true && bluetoothSwitch)
  {
    //Send welcome message to connected device
    if (!sentMessage && SerialBT.connected())
    {
      SerialBT.println("Welcome! Send \"start\" to start or \"clear\" to clear memory or \"end\" to disconnect");
      sentMessage = true;
    }

    //received message from BT
    if (SerialBT.available())
    {
      String s = SerialBT.readString();
      Serial.println("Received BT message: " + s);

      if (s.startsWith("start"))
      {
        SerialBT.println("Start working... :-)");
        for (size_t i = 0; i < EEPROMStore.getMaximalIndex(); i++)
        {
            uint32_t value = EEPROMStore.getValueFromEEPROM(i);
            if (value == 0) break;
            SerialBT.println("Value at index " + String(i) + " is " + String(value));
        }
      }
      else if (s.startsWith("clear"))
      {
        EEPROMStore.clearEEPROM();
        SerialBT.println("Memory is cleared.");
      }
      else if (s.startsWith("end"))
      {
        SerialBT.println("Goodbye... :-)");
        stateSwitch(false);
        break;
      }
      else
      {
        //unexpected message - resend welcome message
        sentMessage = false;
      }
    }
    delay(20);
  }

  delay(500);
  SerialBT.end();
}

//Loop
void loop()
{
  // put your main code here, to run repeatedly:
  if (bluetoothSwitch)
  {
    programWhenBTOn();
  }
  else
  {
    programWhenBTOff();
  }
}
