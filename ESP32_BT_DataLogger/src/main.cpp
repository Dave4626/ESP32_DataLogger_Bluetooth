#include <Arduino.h>
#include "BluetoothSerial.h"
#include "Store.h"
#include "TempData.h"
#include <Wire.h>
#include <RTClib.h>

//Define constants
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2 
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
RTC_DS3231 rtc;

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

void onAlarm()
{
  Serial.println("Alarm occured!");
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

  //RTC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    abort();
  }
  if (rtc.lostPower())
  {
    // this will adjust to the date and time at compilation
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  //we don't need the 32K Pin, so disable it
  rtc.disable32K();
  // Making it so, that the alarm will trigger an interrupt
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);
  // turn off alarm 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  rtc.disableAlarm(2);
  // schedule an alarm 10 seconds in the future
  if (!rtc.setAlarm1(rtc.now() + TimeSpan(0,0,1,0),DS3231_A1_Hour))
  {
    Serial.println("Error, alarm wasn't set!");
  }
  else
  {
    Serial.println("Alarm will happen in 30 seconds!");
  }
}

//Program running when bluetooth flag is off
void programWhenBTOff()
{
  Serial.println("Led is now OFF " + String(millis()));
  uint16_t timeInMinutes = millis() / 1000 / 60;
  TempData t = {Temperature : 1, Humidity : 2, HoursFromCompile : timeInMinutes};
  EEPROMStore.setValueToEEPROM(&t);
  delay(20000);

  // print current time
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);
  Serial.print(date);
  // the value at SQW-Pin (because of pullup 1 means no alarm)
  Serial.print(" SQW: ");
  Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));
  // whether a alarm happened happened
  Serial.print(" Alarm1: ");
  Serial.print(rtc.alarmFired(1));
  // resetting SQW and alarm 1 flag
  // using setAlarm1, the next alarm could now be configurated
  if (rtc.alarmFired(1))
  {
    rtc.clearAlarm(1);
    Serial.println("Alarm cleared");
  }
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
          TempData value = EEPROMStore.getValueFromEEPROM(i);
          uint32_t valueAsNumber = ToUint32(&value);
          if (valueAsNumber == 0)
            break;
          SerialBT.println("Value at index " + String(i) + " is " + GetTempDataAsString(&value));
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
