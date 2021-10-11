#include <Arduino.h>
#include "BluetoothSerial.h"
#include "Store.h"
#include "LoggerData.h"
#include "LoggerConfig.h"
#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"

//Define constants
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2
#define BUTTONPIN 0
#define MYLEDPIN 13
#define DHT11PIN 4

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Global variables
volatile bool bluetoothSwitch = false;
static unsigned long last_interrupt_time = 0;
float tempDHT = 0;
float humiDHT = 0;
BluetoothSerial SerialBT;
Store EEPROMStore;
RTC_DS3231 rtc;
DHT dht(DHT11PIN, DHT11);

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
  digitalWrite(MYLEDPIN, state ? HIGH : LOW);
  digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
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

//on rtc alarm
void onAlarm()
{
  Serial.println("Alarm occured!");
}

//Set next alarm after period from configuration
void setNextAlarm()
{
  rtc.clearAlarm(1);
  Serial.println("Alarm cleared");
  LoggerConfig config = EEPROMStore.getConfig();
  uint32_t period = config.period > 0 ? config.period : 1;

  while (!rtc.setAlarm1(rtc.now() + TimeSpan(period * 60), DS3231_A1_Hour))
  {
    Serial.println("Error, alarm wasn't set!");
    delay(500);
  }

  Serial.println("Alarm will happen in " + String(period * 60) + " seconds!");
}

//Setup
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("START");
  dht.begin();
  pinMode(MYLEDPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(MYLEDPIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
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
    Serial.println("Setup time for RTC after lost power.");
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
  // schedule an alarm
  setNextAlarm();
}

// read data from DHT (temp a humi)
bool readDHT()
{
  tempDHT = 0;
  humiDHT = 0;
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();
  if (isnan(temp) || isnan(humi))
  {
    return false;
  }
  else
  {
    tempDHT = temp;
    humiDHT = humi;
    Serial.println("DHT: " + String(temp) + "C " + String(humi) + "%");
    return true;
  }
}

//Program running when bluetooth flag is off
void programWhenBTOff()
{
  Serial.println("(LOGGER MODE) Led is now OFF " + String(millis()));
  // print current time
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);
  Serial.print(date);
  // the value at SQW-Pin (because of pullup 1 means no alarm)
  Serial.print(" SQW: ");
  Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));
  // whether a alarm happened happened
  Serial.print(" Alarm1: ");
  Serial.println(rtc.alarmFired(1));
  // resetting SQW and alarm 1 flag
  // using setAlarm1, the next alarm could now be configurated
  if (rtc.alarmFired(1))
  {
    time_t time = rtc.now().unixtime();
    while (!readDHT())
    {
      delay(10);
    }
    uint16_t temp = (uint16_t)(tempDHT * 10);
    uint16_t humi = (uint16_t)(humiDHT * 10);
    Serial.println("Data for sensor: " + String(temp / 10.0) + "C, " + String(humi / 10.0) + "%");
    LoggerData t = {Temperature : temp, Humidity : humi, Time : time};
    EEPROMStore.setValueToEEPROM(&t);
    setNextAlarm();
  }
  delay(2000);
}

//Program running when BT is ON
void programWhenBTOn()
{
  SerialBT.begin("ESP32_DATALOGGER_BT");
  Serial.println("(BLUETOOTH MODE) The device started, now you can pair it with bluetooth!");

  bool sentMessage = false;
  while (true && bluetoothSwitch)
  {
    //Send welcome message to connected device
    if (!sentMessage && SerialBT.connected())
    {
      SerialBT.println("ESP32 DATALOGGER BT");
      SerialBT.println("List of commands:");
      SerialBT.println("\"read\" to start reading");
      SerialBT.println("\"period:xxx\" where xxx is time [minutes] to setup logging period");
      SerialBT.println("\"clear\" to clear memory or \"end\" to disconnect");
      SerialBT.println("\"end\" to disconnect and continue in logging");
      sentMessage = true;
    }

    //received message from BT
    if (SerialBT.available())
    {
      String s = SerialBT.readString();
      Serial.println("Received BT message: " + s);

      if (s.startsWith("read")) //READ
      {
        SerialBT.println("INFO: Start reading data:");
        for (size_t i = 0; i < EEPROMStore.getMaximalIndex(); i++)
        {
          LoggerData value = EEPROMStore.getValueFromEEPROM(i);
          if (IsEmpty(&value))
            break;
          SerialBT.println("Index;" + String(i) + ";" + GetLoggerDataAsString(&value));
        }
        SerialBT.println("INFO: End of reading data.");
        sentMessage = false;
      }
      else if (s.startsWith("period")) //SETUP PERIOD
      {
        String periodMinutesString = s.substring(7);
        uint32_t periodMinutes = strtoul(periodMinutesString.c_str(), NULL, 10);
        if (periodMinutes > 0)
        {
          LoggerConfig conf = {period : periodMinutes, _reserved : 0};
          EEPROMStore.setLoggerConfig(&conf);
          SerialBT.println("INFO: Configuration saved. Will be used for next loop or after reboot.");
        }
        else
        {
          SerialBT.println("ERROR: Not valid value.");
        }
        sentMessage = false;
      }
      else if (s.startsWith("clear")) //CLEAR
      {
        SerialBT.println("INFO: Memory will be cleared. Plase wait (it can take some minute)...");
        EEPROMStore.clearEEPROM();
        SerialBT.println("INFO: Memory is cleared.");
        sentMessage = false;
      }
      else if (s.startsWith("end")) //END
      {
        SerialBT.println("Program will continue in logging. Goodbye.");
        stateSwitch(false);
        break;
      }
      else
      {
        //unexpected message - resend welcome message
        SerialBT.println("ERROR: Unexpected message.");
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
