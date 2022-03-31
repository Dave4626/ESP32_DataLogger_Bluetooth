#include <Arduino.h>
#include "BluetoothSerial.h"
#include "Store.h"
#include "LoggerData.h"
#include "LoggerConfig.h"
#include <Wire.h>
#include <RTClib.h>

//#define USE_AHT10
#define USE_SHT31

#ifdef USE_AHT10
#include <Adafruit_AHTX0.h>
#endif
#ifdef USE_SHT31
#include <Adafruit_Sensor.h>
#include "Adafruit_SHT31.h"
#endif

// Define constants
//  the pin that is connected to SQW
#define BUTTONPIN GPIO_NUM_13
#define CLOCK_INTERRUPT_PIN 33
#define WAKE_PIN_BITMASK 0x200000000 // GPIO 33 (2^33 = 8589934592 = 0x200000000) (RTC SQW)

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Global variables
volatile bool bluetoothSwitch = false;
volatile bool logData = false;
static unsigned long last_interrupt_time = 0;
float tempSensor = 0;
float humiSensor = 0;
BluetoothSerial SerialBT;
Store EEPROMStore;
RTC_DS3231 rtc;
TwoWire wire = TwoWire(5);
#ifdef USE_AHT10
Adafruit_AHTX0 aht;
#endif
#ifdef USE_SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31(&wire);
#endif

// switch state
void stateSwitch(bool state)
{
  bluetoothSwitch = state;
  digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
}

// Gets wake up reason
void printWakeupReasonAndSetState()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  // set default state (logger mode)
  stateSwitch(false);
  // set default log state
  logData = false;

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    // wake up from button
    Serial.println("Wakeup caused by external signal using RTC_IO");
    // Set BT state (BT Mode)
    stateSwitch(true);
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    // wake up from RTC
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    // set log data flag to true (will log data)
    logData = true;
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    // reboot
    break;
  }
}

// debounce for button interrupt (only when in BT mode to EXIT from it)
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

// interrupt handler (only when in BT mode to EXIT from it)
void buttonPressed()
{
  if (debounce())
  {
    Serial.println("BUTTON PRESSED");
    detachInterrupt(BUTTONPIN);
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

// on rtc alarm
//  void onAlarm()
//  {
//    Serial.println("Alarm occured!");
//  }

// Set next alarm after period from configuration
void setNextAlarm()
{
  rtc.clearAlarm(1);
  Serial.println("Alarm cleared");
  LoggerConfig config = EEPROMStore.getConfig();
  // get period or default 60sec
  uint32_t periodSec = config.periodMinutes > 0 ? config.periodMinutes * 60 : 60;

  // find next alarm time
  // first create copy of now with alarmStart hour and minute
  DateTime dt = DateTime(rtc.now().year(), rtc.now().month(), rtc.now().day(), config.startHour, config.startMinute, 0);
  DateTime current = rtc.now();
  // then go back to past
  dt = dt - TimeSpan(1, 0, 0, 0);
  // now add period until alarm will be in future
  while (dt < current)
  {
    dt = dt + TimeSpan(periodSec);
  }

  while (!rtc.setAlarm1(dt, DS3231_A1_Hour))
  {
    Serial.println("Error, alarm wasn't set!");
    delay(500);
  }

  Serial.println("Alarm will happen at " + dt.timestamp());
}

// setup RTC and set next alarm
void setupRtc()
{
  // RTC
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
  // we don't need the 32K Pin, so disable it
  rtc.disable32K();
  // Making it so, that the alarm will trigger an interrupt
  // pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
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

// Setup
void setup()
{
  // Serial begin
  Serial.begin(9600);
  delay(2000);

  Serial.println("START");
  pinMode(LED_BUILTIN, OUTPUT);

  // Thermometer init
  while (!wire.begin(GPIO_NUM_4, GPIO_NUM_5))
  {
    Serial.println("Wire cannot begin");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
#ifdef USE_AHT10
  while (!aht.begin(&wire))
  {
    Serial.println("Could not find AHT? Check wiring");
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
  }
#endif
#ifdef USE_SHT31
  while (!sht31.begin(0x44))
  {
    Serial.println("Couldn't find SHT31");
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
  }

  if (sht31.isHeaterEnabled())
    sht31.heater(false);
#endif

  // Print the wakeup reason for ESP32 and set state
  printWakeupReasonAndSetState();

  // setup rtc with first/next alarm
  setupRtc();

  // setup wake up pins
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_ext0_wakeup(BUTTONPIN, 1);

  // other is now up to loop method
  if (bluetoothSwitch == true)
  {
    attachInterrupt(BUTTONPIN, buttonPressed, RISING);
  }
}

// read data from Sensor (temp a humi)
bool readSensor()
{
  tempSensor = -1;
  humiSensor = -1;
#ifdef USE_AHT10
  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature);
  float temp = temperature.temperature;
  float humi = humidity.relative_humidity;
#endif
#ifdef USE_SHT31
  float temp = sht31.readTemperature();
  float humi = sht31.readHumidity();
#endif

  if (isnan(temp) || isnan(humi))
  {
    return false;
  }
  else
  {
    tempSensor = temp;
    humiSensor = humi;
    Serial.println("Thermometer: " + String(temp) + "C " + String(humi) + "%");
    return true;
  }
}

// read Sensor data x times and make average
bool readSensorAverage(uint8_t times = 3)
{
  float temp = 0;
  float humi = 0;
  uint8_t cnt = 0;
  while (cnt < times)
  {
    if (!readSensor())
    {
      return false;
    }
    temp += tempSensor;
    humi += humiSensor;
    cnt++;

    delay(500);
  }
  tempSensor = temp / (float)times;
  humiSensor = humi / (float)times;
  return true;
}

// Turn light off and go to sleep
void goToSleep()
{
  Serial.println("Going to sleep now");
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  esp_deep_sleep_start();
}

// Program running when bluetooth flag is off
void programWhenBTOff()
{
  // log data ony if woke up by RTC alarm
  if (logData)
  {
    // read and log data
    time_t time = rtc.now().unixtime();
    while (!readSensorAverage())
    {
      delay(10);
    }
    uint16_t temp = (uint16_t)(tempSensor * 10);
    uint16_t humi = (uint16_t)(humiSensor * 10);
    Serial.println("Data for sensor: " + String(temp / 10.0) + "C, " + String(humi / 10.0) + "%");
    LoggerData t = {Temperature : temp, Humidity : humi, Time : time};
    EEPROMStore.setValueToEEPROM(&t);

#ifdef USE_SHT31
    // sht31.heater(true);
    // Serial.println("Using sht31 heater, to reduce humidity in sensor");
    // delay(30000);
    // sht31.heater(false);
#endif
  }
  else
  {
    Serial.println("Woke up, but not by RTC (no logging)");
  }
  // Set alarm and go to sleep
  setNextAlarm();
  delay(100);
  goToSleep();
}

// Program running when BT is ON
void programWhenBTOn()
{
  SerialBT.begin("ESP32_DATALOGGER_BT");
  Serial.println("(BLUETOOTH MODE) The device started, now you can pair it with bluetooth!");

  bool sentMessage = false;
  while (true && bluetoothSwitch)
  {
    // Send welcome message to connected device
    if (!sentMessage && SerialBT.connected())
    {
      SerialBT.println("ESP32 DATALOGGER BT");
      SerialBT.println("List of commands: \"help\", \"read\", \"config:HH:MM:period\", \"time:yyyy-MM-DDTHH:mm:ss\", \"clear\", \"end\"");
      sentMessage = true;
    }

    // received message from BT
    if (SerialBT.available())
    {
      String s = SerialBT.readString();
      Serial.println("Received BT message: " + s);

      if (s.startsWith("read")) // READ
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
        // sentMessage = false;
      }
      else if (s.startsWith("config")) // SETUP PERIOD
      {
        Serial.println("received: " + s);
        // expect "config:12:15:360"
        char buf[s.length() - 7];              // len - "config:"
        s.toCharArray(buf, s.length() - 7, 7); // copy to arr part "12:15:360"
        // split
        uint8_t i = 0;
        String split[3];
        const char *splitChar = ":";
        String temp;
        for (temp = strtok(buf, splitChar); temp; temp = strtok(NULL, splitChar))
        {
          split[i] = String(temp);
          i++;
        }
        // convert strings to integers
        uint8_t hh = (uint8_t)strtol(split[0].c_str(), NULL, 10);
        uint8_t mm = (uint8_t)strtol(split[1].c_str(), NULL, 10);
        uint16_t periodMin = (uint16_t)strtol(split[2].c_str(), NULL, 10);
        // check and save
        if (i == 3 && periodMin > 0)
        {
          LoggerConfig conf = {startHour : hh, startMinute : mm, periodMinutes : periodMin, _reserved : 0};
          EEPROMStore.setLoggerConfig(&conf);
          SerialBT.println("INFO: Configuration saved.");
        }
        else
        {
          SerialBT.println("ERROR: Not valid value.");
        }
        // sentMessage = false;
      }
      else if (s.startsWith("clear")) // CLEAR
      {
        SerialBT.println("INFO: Memory will be cleared. Plase wait (it can take some minute)...");
        EEPROMStore.clearEEPROM();
        SerialBT.println("INFO: Memory is cleared.");
        // sentMessage = false;
      }
      else if (s.startsWith("time")) // ADJUST TIME
      {
        // expected time:yyyy-MM-DDTHH:mm:ss
        String timeIsoStr = s.substring(5, 24);
        SerialBT.println("INFO: Set time to " + timeIsoStr);
        rtc.adjust(DateTime(timeIsoStr.c_str()));
        SerialBT.println("INFO: Time is adjusted now to: " + rtc.now().timestamp());
        // sentMessage = false;
      }
      else if (s.startsWith("help")) // HELP
      {
        SerialBT.println("List of commands: \"help\", \"read\", \"config:HH:MM:period\", \"time:yyyy-MM-DDTHH:mm:ss\", \"clear\", \"end\"");
        SerialBT.println("\"help\" to see this :-)");
        SerialBT.println("\"read\" to start reading");
        SerialBT.println("\"config:HH:MM:period\" where HH and MM is reference hour and minute, period is logging period [minutes]");
        SerialBT.println("(note config:12-15-360 means meassure each 6hours at: 12:15, 18:15, 00:15, 06:15, 12:15 etc)");
        SerialBT.println("\"clear\" to clear memory or \"end\" to disconnect");
        SerialBT.println("\"time:yyyy-MM-DDTHH:mm:ss\" to set time");
        SerialBT.println("\"end\" to disconnect and continue in logging");
        LoggerConfig cfg = EEPROMStore.getConfig();
        SerialBT.println("Alarm config " + String(cfg.startHour) + ":" + String(cfg.startMinute) + " period [min] " + String(cfg.periodMinutes));
        SerialBT.println("RTC Time is " + rtc.now().timestamp());
        while (!readSensor())
        {
          delay(20);
        }
        SerialBT.println("First read Temperature: " + String(tempSensor) + " Humidity: " + String(humiSensor));
        SerialBT.println("Reading average temp, please wait...");
        while (!readSensorAverage())
        {
          delay(20);
        }
        SerialBT.println("Average (currently used) Temperature: " + String(tempSensor) + " Humidity: " + String(humiSensor));
      }
      else if (s.startsWith("end")) // END
      {
        SerialBT.println("Program will continue in logging. Goodbye.");
        stateSwitch(false);
        break;
      }
      else
      {
        // unexpected message - resend welcome message
        SerialBT.println("ERROR: Unexpected message.");
        sentMessage = false;
      }
    }
    delay(20);
  }

  // BT off and Switch state
  delay(500);
  SerialBT.end();
  stateSwitch(false);
  // Set alarm and go to sleep
  setNextAlarm();
  delay(100);
  goToSleep();
}

// Loop
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
