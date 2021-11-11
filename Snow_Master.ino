#include <IridiumSBD.h> // http://librarymanager/All#IridiumSBDI2C
#include <time.h>
#include <DS3232RTC.h> // https://github.com/JChristensen/DS3232RTC
#include <OneWire.h> // https://github.com/adafruit/MAX31850_OneWire
#include <DallasTemperature.h> //https://github.com/adafruit/MAX31850_DallasTemp
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h> //https://arduinojson.org/?utm_source=meta&utm_medium=library.properties
#include <StreamUtils.h>

#define VOLTAGE_SENSOR_PIN A0
#define TEMP_PIN 7
#define UTRIG_PIN 13
#define UECHO_PIN 12
#define CS_SD_CARD_PIN 53
#define IridiumSerial Serial1
#define DIAGNOSTICS false // Set to true to see diagnostics

#define SCALE1 Serial2
#define SCALE2 Serial3
// Low = receive | High = transmit
#define SCALE1_MODE 3
#define SCALE2_MODE 4
#define SCALE1_POWER 5
#define SCALE2_POWER 6
#define MODE_SEND HIGH
#define MODE_RECV LOW

IridiumSBD modem(IridiumSerial);
OneWire oneWire(TEMP_PIN);
DallasTemperature temp_sensor(&oneWire);

const float BAD_VAL = -99.99;
const int ALRM_TIME_INTERVAL_HR = 12;
const long DEPTH_OFFSET = 0; //Initial depth with no snow

struct LogData {
  char time[21]; //time of measurements
  float weight1 = BAD_VAL; //weight on scale1
  float weight2 = BAD_VAL; //weight on scale2
  float tempC = BAD_VAL; //temp in celcius
  float snow_depth = BAD_VAL; //snow depth
  float voltage = BAD_VAL; //battery voltage
};

LogData logData;
StaticJsonDocument<128> json_doc;

void setup() {

  // Begin Serial communication at a baudrate of 9600:
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);
  // If we're powering the device by USB, tell the library to
  // relax timing constraints waiting for the supercap to recharge.
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  // For "high current" (battery-powered) applications
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  set_time_from_rock_block();

  SCALE1.begin(9600);
  SCALE1.setTimeout(5000);
  SCALE2.begin(9600);
  SCALE2.setTimeout(5000);

  pinMode(SCALE1_POWER, OUTPUT);
  pinMode(SCALE2_POWER, OUTPUT);
  digitalWrite(SCALE1_POWER, LOW);
  digitalWrite(SCALE2_POWER, LOW);
  pinMode(SCALE1_MODE, OUTPUT);
  pinMode(SCALE2_MODE, OUTPUT);

  pinMode(VOLTAGE_SENSOR_PIN, INPUT);
  pinMode(UTRIG_PIN, OUTPUT);
  pinMode(UECHO_PIN, INPUT);

  // Use this block of code to set time on RTC module
  //tmElements_t tm;
  //tm.Hour = 21;
  //tm.Minute = 12;
  //tm.Second = 30;
  //tm.Day = 10;
  //tm.Month = 11;
  //tm.Year = 2021 - 1970;
  //RTC.write(tm);

  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

  //RTC.setAlarm(alarmType, seconds, minutes, hours, dayOrDate);
  RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, 3, 0);
  RTC.alarm(ALARM_1);

  getSaveAndSendData();
}

void loop()
{
  if ( RTC.alarm(ALARM_1) )  {
    Serial.println("Getting measurements...");
    time_t t = RTC.get();
    int new_alarm_time = (hour(t) + ALRM_TIME_INTERVAL_HR) % 24;
    RTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, new_alarm_time, 0);
    RTC.alarm(ALARM_1);

    getSaveAndSendData();
  }
  //digitalClockDisplay();
  delay(1000);
}

void getSaveAndSendData() {
  Serial.println("Getting measurements...");
  json_doc.clear();
  time_t t = RTC.get();
  int time_size = sprintf(logData.time, "%d-%02d-%02dT%02d:%02d:%02dZ", year(t), month(t), day(t), hour(t), minute(t), second(t));
  logData.weight1 = getScale(1);
  logData.weight2 = getScale(2);
  logData.voltage = get_voltage();
  Serial.print("Current battery voltage is: ");
  Serial.println(logData.voltage);
  logData.tempC = get_temp_celcius();
  Serial.print("Current temp is: ");
  Serial.println(logData.tempC);
  logData.snow_depth = DEPTH_OFFSET - get_snow_depth(logData.tempC);
  Serial.print("Current depth is: ");
  Serial.println(logData.snow_depth);

  create_json_from_data();
  write_data_to_log_file();
  send_data_to_rock_block();
  json_doc.clear();
}

float getScale(int scale) {
  float value = BAD_VAL;
  setSerialMode(MODE_RECV);

  switch (scale) {
    case 1:
      digitalWrite(SCALE1_POWER, HIGH);
      value = readScaleData(SCALE1);
      digitalWrite(SCALE1_POWER, LOW);
      break;
    case 2:
      digitalWrite(SCALE2_POWER, HIGH);
      value = readScaleData(SCALE2);
      digitalWrite(SCALE2_POWER, LOW);
      break;
  }
  return value;
}

float readScaleData(Stream & port) {
  // Allows 5 errors because it will occasionally throw an Empty Input Error
  for (int i = 0; i < 15; i++) {
    setSerialMode(MODE_RECV);

    StaticJsonDocument<64> doc;
    if (port.available() > 5) {

      ReadLoggingStream logging(port, Serial);
      DeserializationError err = deserializeJson(doc, logging);

      if (err == DeserializationError::Ok) {
        const char* status = doc["status"];
        return doc["value"];
      } else if (err == DeserializationError::EmptyInput) {
        // This error is expected b/c deserialze doesn't read the last null byte
        Serial.println("Empty input");
      } else {
        Serial.println("Deserialization error");
      }
    } else {
      Serial.println("Scale data not available. Checking again...");
      delay(1000);
    }
  }
  return BAD_VAL;
}

void setSerialMode(int mode) {
  digitalWrite(SCALE1_MODE, mode);
  digitalWrite(SCALE2_MODE, mode);
  delay(50);
}

float get_voltage() {
  float R1 = 30000.0;
  float R2 = 7500.0;
  uint8_t times = 5;
  float s[times];
  for (uint8_t i = 0; i < times; i++) {
    float vout = (analogRead(VOLTAGE_SENSOR_PIN) * 5.0) / 1024.0;
    s[i] = (vout / (R2 / (R1 + R2)));
  }
  insertSort(s, times);
  if (times & 0x01) return s[times / 2];
  return (s[times / 2] + s[times / 2 + 1]) / 2;
}

float get_temp_celcius() {
  uint8_t times = 5;
  float s[times];
  for (uint8_t i = 0; i < times; i++) {
    temp_sensor.requestTemperatures();
    s[i] = (temp_sensor.getTempFByIndex(0) - 32) * 5 / 9;
    yield();
    delay(50);
  }
  insertSort(s, times);
  if (times & 0x01) return s[times / 2];
  return (s[times / 2] + s[times / 2 + 1]) / 2;
}

float get_snow_depth(float currTemp) {

  unsigned long maxDistanceDurationMicroSec;
  int maxDistanceCm = 900;
  uint8_t times = 15;
  float s[times];
  float speedOfSoundInCmPerMicroSec = 0.03313 + 0.0000606 * currTemp; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
  for (uint8_t i = 0; i < times; i++)
  {
    // Make sure that trigger pin is LOW.
    digitalWrite(UTRIG_PIN, LOW);
    delayMicroseconds(2);
    // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(UTRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(UTRIG_PIN, LOW);

    // Compute max delay based on max distance with 25% margin in microseconds
    maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;

    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
    unsigned long durationMicroSec = pulseIn(UECHO_PIN, HIGH, maxDistanceDurationMicroSec); // can't measure beyond max distance
    s[i] = durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec;
    yield();
    delay(100);
  }
  insertSort(s, times);
  if (times & 0x01) return s[times / 2];
  return (s[times / 2] + s[times / 2 + 1]) / 2;
}

void insertSort(float * array, uint8_t size) {
  uint8_t t, z;
  float temp;
  for (t = 1; t < size; t++)
  {
    z = t;
    temp = array[z];
    while ( (z > 0) && (temp < array[z - 1] ))
    {
      array[z] = array[z - 1];
      z--;
    }
    array[z] = temp;
    yield();
  }
}

void create_json_from_data() {

  json_doc["ts"] = logData.time;

  char weight1_buf[8];
  if (logData.weight1 == BAD_VAL) {
    strcpy(weight1_buf, "NM");
  } else {
    dtostrf(logData.weight1, 4, 3, weight1_buf);
  }
  json_doc["w1"] = weight1_buf;

  char weight2_buf[8];
  if (logData.weight2 == BAD_VAL) {
    strcpy(weight2_buf, "NM");
  } else {
    dtostrf(logData.weight2, 4, 3, weight2_buf);
  }
  json_doc["w2"] = weight2_buf;

  char temp_buf[8];
  dtostrf(logData.tempC, 4, 2, temp_buf);
  json_doc["t"] = temp_buf;

  char snow_buf[8];
  dtostrf(logData.snow_depth, 4, 2, snow_buf);
  json_doc["d"] = snow_buf;

  char voltage_buf[8];
  dtostrf(logData.voltage, 4, 2, voltage_buf);
  json_doc["v"] = voltage_buf;

  serializeJson(json_doc, Serial);
  Serial.println("");
}

bool write_data_to_log_file() {

  Serial.println("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("SD card initialization failed!");
    return false;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myLogDataFile = SD.open("logData.log", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myLogDataFile) {
    Serial.println("Writing to logData.log...");

    int err = serializeJson(json_doc, myLogDataFile); //write data to file
    if (err == 0) {
      Serial.println("Failed to write log data to logData.log");
    } else {
      myLogDataFile.println("");
    }

    // close the file:
    myLogDataFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening logData.log");
  }
}

void set_time_from_rock_block() {

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  int err = modem.begin();
  if (err != ISBD_SUCCESS) {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    modem.sleep();
    return;
  }

  struct tm t;
  err = modem.getSystemTime(t);
  if (err == ISBD_SUCCESS) {
    char buf[32];
    sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d",
            t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    Serial.print(F("Iridium time/date is "));
    Serial.println(buf);
    RTC.set(mktime(&t));
  } else if (err == ISBD_NO_NETWORK) {
    // Did it fail because the transceiver has not yet seen the network?
    Serial.println(F("No network detected.  Waiting 10 seconds."));
  } else {
    Serial.print(F("Unexpected error "));
    Serial.println(err);
  }
  modem.sleep();
  return;

}

void send_data_to_rock_block() {

  // Begin satellite modem operation
  Serial.println("Starting modem...");
  int err = modem.begin();
  if (err != ISBD_SUCCESS) {
    Serial.print("Begin failed: error ");
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println("No modem detected: check wiring.");
    modem.sleep();
    return;
  }

  char buffer[128];
  serializeJson(json_doc, buffer);
  Serial.println(buffer);

  err = modem.sendSBDText(buffer);
  if (err != ISBD_SUCCESS) {
    Serial.print("sendSBDBinary failed: error ");
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println("Try again with a better view of the sky.");
  } else {
    Serial.println("Data sent!");
  }
  modem.sleep();
  return;
}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD * device, char c) {
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD * device, char c) {
  Serial.write(c);
}
#endif

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(' ');
  Serial.print(month());
  Serial.print(' ');
  Serial.print(month());
  Serial.print(' ');
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
