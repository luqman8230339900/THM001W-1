#include <Arduino.h>
///this is test for gitbucket

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h> //mb5tm6crSpbZY8yVNYFF
/////wifi ////
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
WiFiManager wm;
#include <PubSubClient.h>

WebServer Server;
// AutoConnect Portal(Server);
// AutoConnectConfig Config; // Enable autoReconnect supported on v0.9.4
// AutoConnectAux Timezone;
WiFiClient espClient;
PubSubClient client(espClient);
//////////////

/// nokia lcd///
#include "HT1621.h"
#include <Wire.h>
#define lcd_background_pin 12
////////////

////sht-31//////
#include "Adafruit_SHT31.h"
////////////

/// RTC////
#include "RTClib.h"
RTC_DS3231 rtc;
/////

////sd card//
#include "FS.h"
#include "SD.h"
#include "SPI.h"
/////////

#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "cert.h"
//////////////
/// freeROS///////////////////
TaskHandle_t Task1;
TaskHandle_t Task2;
// #define TIME_TO_SLEEP 50
// #define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

//#define TIME_TO_SLEEP
// #define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 10000000 /* Conversion factor for micro seconds to MINUTE */
                                //////////////////
// int TIME_TO_SLEEP  =  60;
// int  temp = 0;

struct str_data
{
  String sensorData;
  String MAC;
  uint32_t unix_time;
  float humi = 0.0;
  int temp = 0;
  int avg_temp_sig = 0;
  int Bpercent = 0;
  // String Bpercent;
  String Alert_temp = "255"; /// 1 represent upper limit // 0 represent lower
  String Alert_humi = "255"; /// 1 represent upper limit // 0 represent lower
} send_str;

// sensorData = "mac=" + MAC + ",date=" + formattedDate + ",time=" + unix_time + ",humidity=" + float(humi) + ",temperature=" + int(temp) + ",wifi=" + int(avg_temp_sig) + ",battery=" + String(Bpercent) + "\r\n";

// int avg_temp_sig = 0;
long sig;

int str_lendata;
int str_lenmac;
int avg_Bp;
int final_humidity;

char copy[255];
String dataString = "";
String formattedDate;
String timeStamp;

// uint32_t unix_time;
int bars;
long rssi;
// String MAC;
char SDdata[255];
char macs[255];

char str_data[255];

// String Bpercent;
float volt;

int final_avg_Ap;
int final_avg_Bp;
int average_sg = 0;

int final_t;

int average_t = 0;
int previous_val_humi = 0;

RTC_DATA_ATTR int portal_check = 0;
RTC_DATA_ATTR int rtc_measure_temp = 0;
RTC_DATA_ATTR int rtc_measure_humi = 0;
RTC_DATA_ATTR int rtc_rssi_signal = 0;
RTC_DATA_ATTR int rtc_battry_adc = 0;
RTC_DATA_ATTR int future_time = 0;
RTC_DATA_ATTR int current_time = 0;
RTC_DATA_ATTR int timer_counter_in_deep_sleep = 0;
RTC_DATA_ATTR int send_battery_value = 0;
RTC_DATA_ATTR int previous_val;
RTC_DATA_ATTR int humidity_previous_val;

bool Mqtt_flag = false;
bool Mqtt_flag1 = false;
bool flag_send_data = false;
bool timer_flag = false;
bool force_send_data_flag = false;
// bool chnage_temp_flag = false;
bool timer_1_hour_flag = false;
#define analogInPin 33 /// battery monitoring pin

/// timer
volatile int interruptCounter;
int totalInterruptCounter;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define TIMer_time 70
//#define timer_uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

#define uS_TO_S_FACTOR 60000000ULL /* Conversion factor for micro seconds to seconds */

// #define TIME_TO_SLEEP 50
// #define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

Adafruit_SHT31 sht31 = Adafruit_SHT31();
HT1621 lcd(25, 26, 27);
const char *mqtt_server = "182.180.77.38";
const char *mqtt_user = "irfan";
const char *mqtt_pass = "irfan";

String FirmwareVer = {
    "0.00"};
#define URL_fw_Version "https://raw.githubusercontent.com/luqman8230339900/THM001W-1/main/version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/luqman8230339900/THM001W-1/main/firmware.bin"

// #define URL_fw_Version "https://raw.githubusercontent.com/programmer131/ESP8266_ESP32_SelfUpdate/master/esp32_ota/bin_version.txt"
// #define URL_fw_Bin "https://raw.githubusercontent.com/programmer131/ESP8266_ESP32_SelfUpdate/master/esp32_ota/fw.bin"

unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousMillis_2 = 0;
const long interval = 10000;
const long mini_interval = 1000;
SemaphoreHandle_t xBinarySemaphore;

// SemaphoreHandle_t mutex_v;

struct cont_phar
{
  int upper_temp = 0;
  int lower_temp = 0;
  int upper_humi = 0;
  int lower_humi = 0;
  int sleep_time = 5;
  int sleep_time_activate;
  int active_set_time = 0;
  int force_send = 0;
  String Time_display = "1641020400";

} sett;

void firmwareUpdate(void)
{
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  // httpUpdate.setLedPin(13, LOW);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}
int FirmwareVersionCheck(void)
{
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure *client = new WiFiClientSecure;

  if (client)
  {
    client->setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is
    HTTPClient https;

    if (https.begin(*client, fwurl))
    { // HTTPS
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      }
      else
      {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete client;
  }

  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer))
    {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    }
    else
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  }
  return 0;
}

void repeatedCall()
{

  if (FirmwareVersionCheck())
  {
    firmwareUpdate();
  }
}
///////////////////////////////////////

/// isr timer
void IRAM_ATTR onTimer()
{

  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void init_sensor_and_lcd()
{

  if (!sht31.begin(0x44))
  {
    Serial.println("Couldn't find SHT31");
  }
  lcd.begin();

  lcd.sendCommand(HT1621::RC256K);
  lcd.sendCommand(HT1621::BIAS_THIRD_4_COM);
  lcd.sendCommand(HT1621::SYS_EN);
  lcd.sendCommand(HT1621::LCD_ON);
}

//////clear lcd////
void clear_lcd()
{
  register uint8_t i;

  for (i = 0; i < 20; i++)
  {
    Serial.println("inside for loop");
    lcd.write(i, 0);
    yield();
  }
}
//////////////////

/////////display temp and humidity//////
void display_temp()
{

  int disp_temp = sht31.readTemperature();
  delay(50);
  if (!isnan(disp_temp))
  { // check if 'is not a number'
    // Serial.print("Temp *C = "); Serial.print(disp_temp); Serial.print("\t\t");
  }
  else
  {
    Serial.println("Failed to read temperature");
  }

  if (disp_temp > -1)
  {
    lcd.write(2, 0b00000000); // 1
  }

  switch (disp_temp)
  {
  case -30: // print -30
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b10111111); // 0

    break;
  }

  case -29: // print -29
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case -28: // print -28

  {

    lcd.write(2, 0b01000000);  //-
                               // lcd.write(3, 0b11111110);  // 2
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11111111); // 8
    break;
  }
  case -27: // print -27
  {

    lcd.write(2, 0b01000000);  //-
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b10001111); // 7
    break;
  }

  case -26: // print -26
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11111011); // 6
    break;
  }

  case -25: // print -25
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11011011); // 5
    break;
  }

  case -24: // print -24
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11000111); // 4
    break;
  }
  case -23: // print -23
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b01011111); // 3
    break;
  }

  case -22: // print -22
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b01111101); // 2
    break;
  }
  case -21: // print -21
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b00000111); // 1
    break;
  }
  case -20: // print -20
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b10111111); // 0
    break;
  }
  case -19: // print 0
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case -18: // print -18
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11111111); // 8
    break;
  }

  case -17: // print -17
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b10001111); // 7
    break;
  }
  case -16: // print -16
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11111011); // 6
    break;
  }
  case -15: // print -15
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11011011); // 5
    break;
  }
  case -14: // print -14
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11000111); // 4
    break;
  }
  case -13: // print -13
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b01011111); // 3
    break;
  }
  case -12: // print -12
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b01111101); // 2
    break;
  }
  case -11: // print -11
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b00000111); // 1
    break;
  }

  case -10: // print -10
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b10111111); // 0
    break;
  }
  case -9: // print -9
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 1
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case -8: // print -8
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11111111); // 8
    break;
  }

  case -7: // print -7
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b10001111); // 7
    break;
  }

  case -6: // print -6
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11111011); // 6

    break;
  }

  case -5: // print -5
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11011011); // 5
    break;
  }
  case -4: // print -4
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11000111); // 4
    break;
  }

  case -3: // print -3
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b01011111); // 3
    break;
  }
  case -2: // print -2
  {

    lcd.write(2, 0b01000000);  // 1
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b01111101); // 2
    break;
  }

  case -1: // print -1
  {

    lcd.write(2, 0b01000000);  // -
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b00000111); // 1
    break;
  }
  case 0: // print 0
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b10111111); // 0
    break;
  }

  case 1: // print 1
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b00000111); // 1
    break;
  }

  case 2: // print 2
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b01111101); // 2
    break;
  }
  case 3: // print 3
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b01011111); // 3
    break;
  }

  case 4: // print 4
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11000111); // 4
    break;
  }

  case 5: // print 5
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11011011); // 5
    break;
  }
  case 6: // print 6
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11111011); // 6
    break;
  }
  case 7: // print 7
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b10001111); // 7
    break;
  }

  case 8: // print 8
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11111111); // 8
    break;
  }
  case 9: // print 9
  {
    lcd.write(3, 0b00000000);  // 0
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case 10: // print 10
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b10111111); // 0
    break;
  }

  case 11: // print 11
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b00000111); // 1
    break;
  }
  case 12: // print 12
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b01111101); // 2
    break;
  }
  case 13: // print 13
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b01011111); // 3
    break;
  }
  case 14: // print 14
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11000111); // 4
    break;
  }
  case 15: // print 15
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11011011); // 5
    break;
  }
  case 16: // print 16
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11111011); // 6
    break;
  }

  case 17: // print 17
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b10001111); // 7
    break;
  }
  case 18: // print 18
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11111111); // 8
    break;
  }
  case 19: // print 19
  {

    lcd.write(3, 0b00000110);  // 1
    lcd.write(11, 0b11011111); // 9
    break;
  }
  case 20: // print 20
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b10111111); // 0
    break;
  }

  case 21: // print 21
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b00000111); // 1
    break;
  }

  case 22: // print 22
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b01111101); // 2
    break;
  }
  case 23: // print 23
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b01011111); // 3
    break;
  }

  case 24: // print 24
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11000111); // 4
    break;
  }

  case 25: // print 25
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11011011); // 5
    break;
  }
  case 26: // print 26
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11111011); // 6
    break;
  }

  case 27: // print 27
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b10001111); // 7
    break;
  }
  case 28: // print 28

  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11111111); // 8
    break;
  }
  case 29: // print 29
  {

    lcd.write(3, 0b01111100);  // 2
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case 30: // print 30
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b10111111); // 0
    break;
  }

  case 31: // print 31
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b00000111); // 1
    break;
  }

  case 32: // print 32
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b01111101); // 2
    break;
  }

  case 33: // print 33
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b01011111); // 3
    break;
  }

  case 34: // print 34
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b11000111); // 4

    break;
  }

  case 35: // print 35
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b11011011); // 5

    break;
  }

  case 36: // print 36
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b11111011); // 6
    break;
  }

  case 37: // print 37
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b10001111); // 7
    break;
  }

  case 38: // print 38
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b11111111); // 8
    break;
  }

  case 39: // print 39
  {

    lcd.write(3, 0b01011110);  // 3
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case 40: // print 40
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b10111111); // 0
    break;
  }

  case 41: // print 41
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b00000111); // 1
    break;
  }

  case 42: // print 42
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b01111101); // 2
    break;
  }

  case 43: // print 43
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b01011111); // 3
    break;
  }

  case 44: // print 44
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b11000111); // 4
    break;
  }

  case 45: // print 45
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b11011011); // 5
    break;
  }

  case 46: // print 46
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b11111011); // 6
    break;
  }
  case 47: // print 47
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b10001111); // 7
    break;
  }

  case 48: // print 48
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b11111111); // 8
    break;
  }

  case 49: // print 49
  {

    lcd.write(3, 0b11000110);  // 4
    lcd.write(11, 0b11011111); // 9
    break;
  }

  case 50: // print 50
  {

    lcd.write(3, 0b11011010);  // 5
    lcd.write(11, 0b10111111); // 0
    break;
  }
  }
}

void display_humidity()
{
  float disp_humidity = sht31.readHumidity();
  delay(50);
  if (!isnan(disp_humidity))
  { // check if 'is not a number'
    // Serial.print("Hum. % = "); Serial.println(disp_humidity);
  }
  else
  {

        Serial.println("Failed to read humidity");
  }

  int temp1; // a;
  int temp2; // b;
  temp1 = disp_humidity;

  temp2 = disp_humidity * 10 - temp1 * 10;
  int after_point = 0;  // average_Ap = 0; // after ppoint
  int before_point = 0; // average_Bp = 0; // before point
  int avg_Ap;           // final_avg_Ap;
  // int avg_Bp;           /// final_avg_Bp;

  for (int avg = 0; avg <= 20; avg++)
  {
    after_point = after_point + temp2;
    before_point = before_point + temp1;
  }
  avg_Ap = after_point / 20;
  avg_Bp = before_point / 20;

  final_humidity = avg_Bp;

  if (final_humidity > 0 && final_humidity < 100)
  {

    rtc_measure_humi = final_humidity;
  }

  if (final_humidity < 0 || final_humidity > 100)
  {
    final_humidity = rtc_measure_humi;
  }

  delay(20);
  // Serial.println(b);
  if (avg_Ap == 1)
  {
    lcd.write(14, 0b00000111); // 1
  }

  if (avg_Ap == 2)
  {
    lcd.write(14, 0b01111101); // 2
  }
  if (avg_Ap == 3)
  {
    lcd.write(14, 0b01011111); // 3
  }
  if (avg_Ap == 4)
  {
    lcd.write(14, 0b11000111); // 4
  }
  if (avg_Ap == 5)
  {
    lcd.write(14, 0b11011011); // 5
  }
  if (avg_Ap == 6)
  {
    lcd.write(14, 0b11111011); // 6
  }
  if (avg_Ap == 7)
  {
    lcd.write(14, 0b10001111); // 7
  }
  if (avg_Ap == 8)
  {
    lcd.write(14, 0b11111111); // 8
  }
  if (avg_Ap == 9)
  {
    lcd.write(14, 0b11011111); // 9
  }
  if (avg_Ap == 0)
  {
    lcd.write(14, 0b10111111); // 0
  }

  switch (final_humidity)
  {

  case 0: // print 0
  {

    lcd.write(13, 0b10111111); // 0
    break;
  }

  case 1: // print 1
  {

    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 2: // print 2
  {

    lcd.write(13, 0b01111101); // 2

    break;
  }
  case 3: // print 3
  {

    lcd.write(13, 0b01011111); // 3

    break;
  }

  case 4: // print 4
  {

    lcd.write(13, 0b11000111); // 4
    break;
  }

  case 5: // print 5
  {

    lcd.write(13, 0b11011011); // 5

    break;
  }
  case 6: // print 6
  {

    lcd.write(13, 0b11111011); // 6

    break;
  }
  case 7: // print 7
  {

    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 8: // print 8
  {

    lcd.write(13, 0b11111111); // 8

    break;
  }
  case 9: // print 9
  {

    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 10: // print 10
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 11: // print 13
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b00000111); // 1

    break;
  }
  case 12: // print 12
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b01111101); // 2

    break;
  }
  case 13: // print 13
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b01011111); // 12

    break;
  }
  case 14: // print 14
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b11000111); // 4

    break;
  }
  case 15: // print 15
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b11011011); // 5

    break;
  }
  case 16: // print 16
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b11111011); // 6

    break;
  }

  case 17: // print 17
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b10001111); // 7

    break;
  }
  case 18: // print 18
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b11111111); // 8

    break;
  }
  case 19: // print 19
  {

    lcd.write(12, 0b00000110); // 1
    lcd.write(13, 0b11011111); // 9

    break;
  }
  case 20: // print 20
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 21: // print 21
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 22: // print 22
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b01111101); // 2

    break;
  }
  case 23: // print 23
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b01011111); // 12

    break;
  }

  case 24: // print 24
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 25: // print 25
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b11011011); // 5

    break;
  }
  case 26: // print 26
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b11111011); // 6

    break;
  }

  case 27: // print 27
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b10001111); // 7

    break;
  }
  case 28: // print 28

  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b11111111); // 8

    break;
  }
  case 29: // print 29
  {

    lcd.write(12, 0b01111100); // 2
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 30: // print 30
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 31: // print 31
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 32: // print 32
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 33: // print 33
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b01011111); // 12

    break;
  }

  case 34: // print 34
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 35: // print 35
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b11011011); // 5

    break;
  }

  case 36: // print 36
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b11111011); // 6

    break;
  }

  case 37: // print 37
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 38: // print 38
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 39: // print 39
  {

    lcd.write(12, 0b01011110); // 12
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 40: // print 40
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 41: // print 41
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 42: // print 42
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 43: // print 43
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b01011111); // 12

    break;
  }

  case 44: // print 44
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 45: // print 45
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b11011011); // 5

    break;
  }

  case 46: // print 46
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b11111011); // 6

    break;
  }
  case 47: // print 47
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 48: // print 48
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 49: // print 49
  {

    lcd.write(12, 0b11000110); // 4
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 50: // print 50
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 51: // print 51
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 52: // print 52
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 53: // print 53
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b01011111); // 3

    break;
  }

  case 54: // print 54
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 55: // print 55
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b11011011); // 5

    break;
  }
  case 56: // print 56
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b11111011); // 6

    break;
  }

  case 57: // print 57
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b10001111); // 7

    // for(int i; i <=32; i ++)
    // {
    //  lcd.write(i , 0b10111111); //0
    //  Serial.println(i);
    // delay (2000);
    // }

    break;
  }

  case 58: // print 58
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 59: // print 59
  {

    lcd.write(12, 0b11011010); // 5
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 60: // print 60
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 61: // print 61
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 62: // print 62
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 63: // print 63
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b01011111); // 3
    break;
  }

  case 64: // print 64
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 65: // print 65
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b11011011); // 5

    break;
  }

  case 66: // print 66
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b11111011); // 6

    break;
  }
  case 67: // print 67
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 68: // print 68
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 69: // print 69
  {

    lcd.write(12, 0b11111010); // 6
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 70: // print 70
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 71: // print 71
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 72: // print 72
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 73: // print 73
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b01011111); // 3

    break;
  }

  case 74: // print 74
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 75: // print 75
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b11011011); // 5

    break;
  }

  case 76: // print 76
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b11111011); // 6

    break;
  }
  case 77: // print 77
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 78: // print 78
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 79: // print 79
  {

    lcd.write(12, 0b10001110); // 7
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 80: // print 80
  {

    lcd.write(12, 0b11111110); // 6
    lcd.write(13, 0b10111111); // 0

    break;
  }

  case 81: // print 81
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b00000111); // 1

    break;
  }

  case 82: // print 82
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b01111101); // 2

    break;
  }

  case 83: // print 83
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b01011111); // 3

    break;
  }

  case 84: // print 84
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b11000111); // 4

    break;
  }

  case 85: // print 85
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b11011011); // 5

    break;
  }

  case 86: // print 86
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b11111011); // 6

    break;
  }
  case 87: // print 87
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b10001111); // 7

    break;
  }

  case 88: // print 88
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b11111111); // 8

    break;
  }

  case 89: // print 89
  {

    lcd.write(12, 0b11111110); // 8
    lcd.write(13, 0b11011111); // 9

    break;
  }

  case 90: // print 90
  {

    lcd.write(12, 0b11011110); // 9
    lcd.write(13, 0b10111111); // 0

    break;
  }
  }
}

////wifi signals//////
void wifi_rssi_sig()
{
  int temp_battery = 0;
  int bat = 0;
  int bat1 = 0;
  int average_sg = 0;
  // Serial.println("battry function");
  //  bat = (analogRead(analogInPin));
  //  bat1 = map(bat, 0, 4095, 0, 255);

  for (int i = 0; i <= 10; i++)
  {
    bat = (analogRead(analogInPin));
    bat1 = map(bat, 0, 4095, 0, 255);
    temp_battery = temp_battery + bat1;
  }
  bat1 = temp_battery / 11;
  Serial.print("ADC reading ==");
  Serial.print(bat1);

  for (int sig_avg = 0; sig_avg <= 20; sig_avg++)
  {
    rssi = WiFi.RSSI();
    average_sg = average_sg + rssi;
  }

  send_str.avg_temp_sig = average_sg / 20;
  Serial.print("signals == ");
  Serial.println(send_str.avg_temp_sig);

  if ((send_str.avg_temp_sig != rtc_rssi_signal) || (rtc_battry_adc != bat1))
  {
    rtc_rssi_signal = send_str.avg_temp_sig;
    rtc_battry_adc = bat1;

    if (send_str.avg_temp_sig >= -50 && send_str.avg_temp_sig < -10)
    {
      Serial.print("full signals");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11111111); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01111111); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00111111); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00011111); /// 0b00001111
      }
    }

    if (send_str.avg_temp_sig < -50 && send_str.avg_temp_sig >= -55) //-54
    {
      Serial.println("signals 3 bars");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11111110); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01111110); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00111110); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00011110); /// 0b00001111
      }
    }

    if (send_str.avg_temp_sig < -55 && send_str.avg_temp_sig >= -70) //-58
    {

      Serial.println("signals 2 bars");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11111100); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01111100); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00111100); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00011100); /// 0b00001111
      }
    }

    if (send_str.avg_temp_sig < -70 && send_str.avg_temp_sig >= -80)
    {
      Serial.println("signals 2 bars");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11111100); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01111100); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00111100); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00011100); /// 0b00001111
      }
    }

    if (send_str.avg_temp_sig < -80 && send_str.avg_temp_sig >= -90)
    {
      Serial.println("signals 2 bars");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11111100); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01111100); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00111100); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00011100); /// 0b00001111
      }
    }

    if (send_str.avg_temp_sig > -9 || send_str.avg_temp_sig < -90)
    {
      Serial.println("no signals");

      if (bat1 >= 173 && bat1 <= 240) // 4.2 -- 4.3v
      {
        send_str.Bpercent = 100;
        lcd.write(15, 0b11110000); /// 0b00001111
      }

      if (bat1 >= 160 && bat1 < 173) // 3.8 -- 4.2v
      {
        send_str.Bpercent = 66;

        lcd.write(15, 0b01110000); /// 0b00001111
      }

      //////3.4 -------  3.6    --  1 segment ////
      if (bat1 >= 144 && bat1 < 160)
      {
        send_str.Bpercent = 33;
        lcd.write(15, 0b00110000); /// 0b00001111
      }
      if (bat1 > 240 || bat1 < 144)
      {
        send_str.Bpercent = 0;
        lcd.write(15, 0b00010000); /// 0b00001111
      }
    }

    send_battery_value = send_str.Bpercent;
  }
}
//////////////////////

///////////////autoconnect functions///////////
void sht31_dsclock_Sleep(int sht_address, int ds_clock_adress)
{
  const uint8_t CTRL_MEAS_REG = 0xF4;
  // BME280 Register 0xF4 (control measurement register) sets the device mode, specifically bits 1,0
  // The bit positions are called 'mode[1:0]'. See datasheet Table 25 and Paragraph 3.3 for more detail.
  // Mode[1:0]  Mode
  //    00      'Sleep'  mode
  //  01 / 10   'Forced' mode, use either '01' or '10'
  //    11      'Normal' mode
  Serial.println("sht31 to Sleep mode...");
  Wire.beginTransmission(sht_address);
  Wire.requestFrom(sht_address, 1);
  uint8_t value = Wire.read();
  value = (value & 0xFC) + 0x00;      // Clear bits 1 and 0
  Wire.write((uint8_t)CTRL_MEAS_REG); // Select Control Measurement Register
  Wire.write((uint8_t)value);         // Send 'XXXXXX00' for Sleep mode
  Wire.endTransmission();

  Serial.println("ds_clock to Sleep mode...");
  Wire.beginTransmission(sht_address);
  Wire.requestFrom(sht_address, 1);
  uint8_t val = Wire.read();
  value = (val & 0xFC) + 0x00;        // Clear bits 1 and 0
  Wire.write((uint8_t)CTRL_MEAS_REG); // Select Control Measurement Register
  Wire.write((uint8_t)val);           // Send 'XXXXXX00' for Sleep mode
  Wire.endTransmission();
}
////////////////////////////////////////////////

////////sd card//////////////
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path))
  {
    Serial.println("File deleted");
  }
  else
  {
    Serial.println("Delete failed");
  }
}

void createDir(fs::FS &fs, const char *path)
{
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path))
  {
    Serial.println("Dir created");
  }
  else
  {
    Serial.println("mkdir failed");
  }
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

void readFile(fs::FS &fs, const char *path)
{
  // Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (file)
  {
    // Serial.println("open file for reading");
    //         return;

    // Serial.print("Read from file: ");
    while (file.available())
    {
      flag_send_data = true;
      String data1 = file.readStringUntil('\n');
      Serial.println("read from sd card");
      Serial.println(data1);
      data1.toCharArray(SDdata, 255);
      // Serial.println("while sendig data from sd card");
      //  Serial.println("macadress= ");
      //  Serial.println(macs);
      Serial.println(SDdata);
      client.publish(macs, SDdata);
      delay(100);

      // if (Mqtt_flag1 == true)
      // {
      //   client.publish(macs, SDdata);
      //   delay(10);
      //   Serial.println("publish data done");
      //  }

      // else
      //   break;
      // client.publish("amq.topic", SDdata);
      //  client.publish("48:3F:DA:4F:D7:63", SDdata);
    }

    if (Mqtt_flag1 == true)
    {
      delay(1);
      Serial.println("delete files");
      deleteFile(SD, "/smart sensing/hello.txt");
      flag_send_data = false;
    }
  }
  else
    Serial.println("not open file for reading");
}
////////////////////////////////////

void my_disconnect()
{

  WiFi.persistent(false);
  WiFi.disconnect();
  WiFi.persistent(true);
}

class FloatParameter : public WiFiManagerParameter
{
public:
  FloatParameter(const char *id, const char *placeholder, float value, const uint8_t length = 10)
      : WiFiManagerParameter("")
  {
    init(id, placeholder, String(value).c_str(), length, "", WFM_LABEL_BEFORE);
  }

  float getValue()
  {
    return String(WiFiManagerParameter::getValue()).toFloat();
  }
};

class IntParameter : public WiFiManagerParameter
{
public:
  IntParameter(const char *id, const char *placeholder, long value, const uint8_t length = 10)
      : WiFiManagerParameter("")
  {
    init(id, placeholder, String(value).c_str(), length, "", WFM_LABEL_BEFORE);
  }

  long getValue()
  {
    return String(WiFiManagerParameter::getValue()).toInt();
  }
};

class StringParameter : public WiFiManagerParameter
{
public:
  StringParameter(const char *id, const char *placeholder, String value, const uint8_t length = 10)
      : WiFiManagerParameter("")
  {
    init(id, placeholder, String(value).c_str(), length, "", WFM_LABEL_BEFORE);
  }

  String getValue()
  {
    // return String(WiFiManagerParameter::getValue()).toCharArray();
        return String(WiFiManagerParameter::getValue()).c_str();
  }
};


void force_to_send_data()
{
  DateTime time = rtc.now();
  DateTime future(time + TimeSpan(0, 0, 13, 0));

  if (timer_counter_in_deep_sleep == 0)
  {
    Serial.println("set device time Interval");
    // current_time = time.minute();
    current_time = time.unixtime();
    Serial.println("current minutes= ");
    Serial.println(current_time);
    // future_time =  future.minute();
    future_time = future.unixtime();
    Serial.println("Future minutes= ");
    Serial.println(future_time);
    timer_counter_in_deep_sleep++;
  }

  if (timer_counter_in_deep_sleep > 0)
  {

    if (current_time > future_time)
    {

      Serial.println("Current time == Future minutes= Success ");
      timer_counter_in_deep_sleep = 0;
      timer_1_hour_flag = true;
    }
    else
    {

      Serial.println("time not yet reached to future time ");
      // current_time = time.minute();
      current_time = time.unixtime();
      Serial.print("current minutes= ");
      Serial.println(current_time);
      Serial.print("Future minutes= ");
      Serial.println(future_time);
    }
  }
}

void init_wifi_portal()
{

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
                       // WiFiManager
                       // Local intialization. Once its business is done, there is no need to keep it around

  // reset settings - for testing

  wm.resetSettings();

  // wm.resetSettings();
  //    lcd.write(1, 0b00000000);
  //  lcd.write(2, 0b00000000);

  clear_lcd();

  delay(500);
  lcd.write(3, 0b00100000);
  lcd.write(11, 0b00100000);
  delay(500);
  lcd.write(3, 0b00000000);
  lcd.write(11, 0b00000000);
  delay(500);
  lcd.write(3, 0b00100000);
  lcd.write(11, 0b00010000);
  delay(500);
  lcd.write(3, 0b00000000);
  lcd.write(11, 0b00000000);
  delay(500);
  lcd.write(3, 0b00100000);
  lcd.write(11, 0b00010000);

  DateTime time = rtc.now();
  sett.Time_display = time.unixtime();
        // send_str.unix_time = time.unixtime();


      delay(500);


  // IntParameter param_temp_upper("temp_u", "Temperature upper limit", sett.upper_temp);
  // IntParameter param_temp_lower("temp_l", "Temperature lower limit", sett.lower_temp);

  // IntParameter param_humi_upper("humi_U", "Humidity upper limit", sett.upper_humi);
  // IntParameter param_humi_lower("humi_L", "Humidity lower limit", sett.lower_humi);

  IntParameter param_device_sleep_time("sleep_time", "Send data in minutes ?", sett.sleep_time);
  IntParameter param_device_sleep_time_activate("set_sleep_time", "sleep_time_Activate/ 1/0 ?", sett.sleep_time_activate);
  IntParameter param_force_send_data_activate("force_send", "Forcefully send data 1/0 ?", sett.force_send);

  StringParameter param_display_time("sett_Time_display", "Current Unix time ", sett.Time_display);

  IntParameter param_activate_set_time("activate_set_time", "activate set time 1/0 ?", sett.active_set_time);

  //IntParameter param_set_clock_time("set_time", "Enter time in unix", sett.set_time);

  // wm.addParameter(&param_temp_upper);
  // wm.addParameter(&param_temp_lower);

  // wm.addParameter(&param_humi_upper);
  // wm.addParameter(&param_humi_lower);

  wm.addParameter(&param_device_sleep_time);
  wm.addParameter(&param_device_sleep_time_activate);
  wm.addParameter(&param_force_send_data_activate);
  wm.addParameter(&param_display_time);
   wm.addParameter(&param_activate_set_time);


  wm.setTitle("Device Configuration");
  wm.setHostname("THM 001 W");
  wm.setDarkMode(true);
  wm.setAPStaticIPConfig(IPAddress(10, 0, 1, 1), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));
  // wm.setSTAStaticIPConfig(IPAddress(10, 0, 1, 1), IPAddress(8, 8, 8, 8), IPAddress(255, 255, 255, 0), IPAddress(8,8,8,8));

  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  // wm.setAPCallback(configModeCallback);
  wm.setTimeout(80);
  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration

  if (!wm.autoConnect())
  {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep.

    if (WiFi.getMode() & WIFI_AP)
    {
      WiFi.softAPdisconnect(true);
      WiFi.enableAP(false);
    }
    // ESP.restart();
    // delay(1000);
  }
  //  strncpy(sett.upper_temp, str_temp_upper.getValue(), 10);
  //  sett.upper_temp[9] = '\0';
  // strncpy(sett.lower_temp, str_temp_upper.getValue(), 10);
  //   sett.lower_temp[9] = '\0';

  // sett.upper_temp = param_temp_upper.getValue();
  // sett.lower_temp = param_temp_lower.getValue();

  // sett.upper_humi = param_humi_upper.getValue();
  // sett.lower_humi = param_humi_lower.getValue();

  sett.sleep_time = param_device_sleep_time.getValue();
  sett.sleep_time_activate = param_device_sleep_time_activate.getValue();
  sett.force_send = param_force_send_data_activate.getValue();
  sett.Time_display = param_display_time.getValue();
  sett.active_set_time = param_activate_set_time.getValue();
  //sett.active_set_time = param_activate_set_time.getValue();

  // Serial.print("upper limit temp =  ");
  // Serial.println(sett.upper_temp);
  // Serial.print("lower upper limit temp = ");
  // Serial.println(sett.lower_temp);

  // Serial.print("upper limit humi =  ");
  // Serial.println(sett.upper_humi);
  // Serial.print("lower limit humi= ");
  // Serial.println(sett.lower_humi);

  Serial.print("sleep time_activation= ");
  Serial.println(sett.sleep_time_activate);

  // EEPROM.put(0, sett.upper_temp);
  // EEPROM.put(15, sett.lower_temp);

  // EEPROM.put(25, sett.upper_humi);
  // EEPROM.put(35, sett.lower_humi);

  EEPROM.put(45, sett.sleep_time);
  EEPROM.put(55, sett.sleep_time_activate);
  EEPROM.put(65, sett.force_send);
  EEPROM.put(75, sett.sleep_time_activate);  //  0 / 1
  EEPROM.commit();
  Serial.println("save readings in EEprom ");

  Serial.println("connected...yeey ");

  wifi_rssi_sig();
  rssi = WiFi.RSSI();
  Serial.println(rssi);

  // MAC = WiFi.macAddress();
  // client.setServer(mqtt_server, 1883);
  // SD.begin(5);
  // delay(1);
  // createDir(SD, "/smart sensing");
  // WiFi.mode(WIFI_OFF);

  // WiFi.disconnect();
}

void timer_for_one_hour()
{
  int mint = 0;
  int hours = 0;

  DateTime time = rtc.now();
  mint = time.minute();
  // Serial.print("mint=  ");
  // Serial.println(mint);

  hours = time.hour();
  // Serial.print("hour=  ");
  // Serial.println(hours);

  //  send_str.unix_time = time.unixtime();
  //  Serial.print(send_str.unix_time);

  // if (hours == 4 && mint == 1)
  // {
  //   timer_1_hour_flag = true;
  // }

  // if (mint == 15 || mint == 30 || mint == 45 || mint == 00)
  // {
  //   timer_1_hour_flag = true;
  //   Serial.println("Timer 15 minute complete");
  // }
  // else
  // {

  //   timer_1_hour_flag = false;
  // }
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(wm.getWiFiSSID(true).c_str(), wm.getWiFiPass(true).c_str());
}

void Task1code(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    // vTaskDelay(300);
    display_temp();
    display_humidity();
    // timer_for_one_hour();

    xSemaphoreGive(xBinarySemaphore);
    vTaskDelay(500);
  }
}

// Task2code:
void Task2code(void *pvParameters)
{
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {

    bool chnage_temp_flag = false;
    bool chnage_humidity_flag = false;

    int temp_humidity = 0;

    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

    int t = sht31.readTemperature();
    int h = sht31.readHumidity();

    for (int i = 0; i <= 20; i++)
    {
      temp_humidity = temp_humidity + h;
    }
    h = temp_humidity / 20;

    if (!isnan(t))
    { // check if 'is not a number'
      // Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
    }
    else
    {
      t = previous_val;
      Serial.println("Failed to read temperature in Task 2");
    }

    if (t < -50)
    {
      t = previous_val;
    }
    if (!isnan(h))
    { // check if 'is not a number'

      // Serial.print("Hum. % = "); Serial.println(h);
    }
    else
    {
      Serial.println("Failed to read humidity in Task 2");
    }

    if (t != previous_val)
    {
      chnage_temp_flag = true;
      Serial.print("temp change");
      Serial.println(t);
      // previous_val = t;

      if (t > 80 || t < -60)
      {

        t = previous_val;
      }
      if (t > -40 && t < 100)
      {
        previous_val = t;
      }

      // if (t > sett.upper_temp)
      // {
      //   Serial.println("Upper temperature limit exceed");
      //   send_str.Alert_temp = "1";
      // }
      // else
      // {
      //   send_str.Alert_temp = "255";
      // }

      // if (t < sett.lower_temp)
      // {
      //   Serial.println("lower temperature limit exceed");
      //   send_str.Alert_temp = "0";
      // }
      // else
      // {
      //   send_str.Alert_temp = "255";
      // }

      // if (h > sett.upper_humi)
      // {
      //   Serial.println("Upper humidity limit exceed");
      //   send_str.Alert_humi = "1";
      // }
      // else
      // {

      //   send_str.Alert_humi = "255";
      // }

      // if (h < sett.lower_humi)
      // {
      //   Serial.println("lower humidity limit exceed");
      //   send_str.Alert_humi = "0";
      // }
      // else
      // {

      //   send_str.Alert_humi = "255";
      // }
    }

    if (h != humidity_previous_val)
    {
      chnage_humidity_flag = true;
      Serial.print("Humidity change");
      Serial.print(h);
      // humidity_previous_val = h;
      if (h > 0 && h < 94)
      {

        humidity_previous_val = h;
      }

      if (h < 0 || h > 95)
      {
        h = humidity_previous_val;
      }
    }

    xSemaphoreGive(xBinarySemaphore);
    delay(500);

    if (chnage_temp_flag == true || timer_1_hour_flag == true || chnage_humidity_flag == true || force_send_data_flag)
    {

      WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
      esp_wifi_start();

      if (portal_check == 0)
      {
        esp_wifi_start();
        vTaskDelay(100);
        DateTime time = rtc.now();
        // formattedDate = (time.timestamp(DateTime::TIMESTAMP_DATE));
        // timeStamp = (time.timestamp(DateTime::TIMESTAMP_TIME));
        send_str.unix_time = time.unixtime();

        Serial.print("Unix time = ");
        Serial.println(send_str.unix_time);

        // sensorData = "mac=" + MAC + ",date=" + formattedDate + ",time=" + unix_time + ",humidity=" + float(humi) + ",temperature=" + int(temp) + ",wifi=" + int(avg_temp_sig) + ",battery=" + String(Bpercent) + "\r\n";
        // send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" +   final_humidity + ",temperature=" + t + ",wifi=" + send_str.avg_temp_sig + ",battery=" + send_str.Bpercent + ",software_ver=" + FirmwareVer + ",alert_temp=" + send_str.Alert_temp + +",alert_humidity=" + send_str.Alert_humi + "\r\n";

        send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + final_humidity + ",temperature=" + t + ",wifi=" + rtc_rssi_signal + ",battery=" + send_battery_value + "\r\n";

        Serial.println(send_str.sensorData);
        vTaskDelay(100);
        str_lendata = send_str.sensorData.length() + 1;
        str_lenmac = send_str.MAC.length() + 1;

        // char macs[str_lenmac];
        char copy[str_lendata];

        send_str.MAC.toCharArray(macs, str_lenmac);         //// find length
        send_str.sensorData.toCharArray(copy, str_lendata); //// mac to char
        // vTaskDelay(500);
      }

      Serial.println("wifi connection start");
      vTaskDelay(1000);

      Serial.println(wm.getWiFiSSID(true).c_str());
      Serial.println(wm.getWiFiPass(true).c_str());
      int ct = 1000;

      while (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("tring to connect wifi");
        // WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
        // WiFi.reconnect();
        WiFi.begin(wm.getWiFiSSID(true).c_str(), wm.getWiFiPass(true).c_str());
        Serial.println("configure ssid, password");

        ct = ct + 100;
        if (ct > 1100)
        {
          break;
        }

        vTaskDelay(7000);
      }

      vTaskDelay(50);
      if (WiFi.status() == WL_CONNECTED)
      {

        Serial.println("wifi connected");

        send_str.MAC = WiFi.macAddress();

        DateTime time = rtc.now();
        send_str.unix_time = time.unixtime();

        //-> check for firmware
        // if (timer_1_hour_flag == true) // this mode
        // {
        //   repeatedCall();
        // }
        // Serial.print(" Active fw version:");
        // Serial.println(FirmwareVer);

        // send_str.sensorData = "mac=" + MAC + ",date=" + formattedDate + ",time=" + unix_time + ",humidity=" + float(humi) + ",temperature=" + int(temp) + ",wifi=" + int(avg_temp_sig) + ",battery=" + String(Bpercent) + "\r\n";

        // send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + h + ",temperature=" + t+ ",wifi=" + send_str.avg_temp_sig + ",battery=" + send_str.Bpercent + ",alert_temp=" + send_str.Alert_temp + +",alert_humidity=" + send_str.Alert_humi + "\r\n";

        //  send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + final_humidity + ",temperature=" + t + ",wifi=" + send_str.avg_temp_sig + ",battery=" + send_str.Bpercent + ",software_ver=" + FirmwareVer + ",alert_temp=" + send_str.Alert_temp + +",alert_humidity=" + send_str.Alert_humi + "\r\n";
        wifi_rssi_sig();
        send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + final_humidity + ",temperature=" + t + ",wifi=" + rtc_rssi_signal + ",battery=" + send_battery_value + "\r\n";

        // Serial.println(avg_temp_sig);
        // Serial.println(sensorData);
        vTaskDelay(100);
        str_lendata = send_str.sensorData.length() + 1;
        str_lenmac = send_str.MAC.length() + 1;

        char macs[str_lenmac];
        char copy[str_lendata];

        send_str.MAC.toCharArray(macs, str_lenmac);         //// find length
        send_str.sensorData.toCharArray(copy, str_lendata); //// mac to char
        Serial.println(copy);
        send_str.sensorData.toCharArray(copy, str_lendata);     //// mac to char
        send_str.sensorData.toCharArray(str_data, str_lendata); //// mac to char

        // vTaskDelay(1000);
        if (!client.connected())
        {
          Serial.println("trying to connect with client");
          // client.setKeepAlive(60); // setting keep alive to 90 seconds luqman16
          client.loop();
          client.connect("THM 001 W", mqtt_user, mqtt_pass);
          // client.publish(macs, copy);
          vTaskDelay(100);
          // Mqtt_flag = true;
          // Mqtt_flag1 = true;
        }
        if (client.connected())
        {
          Serial.println("client connected");
          Serial.print("data == ");
          Serial.println(copy);
          Mqtt_flag = true;
          Mqtt_flag1 = true;
          Serial.println("RabbitMQ connected");
          readFile(SD, "/smart sensing/hello.txt");

          Serial.println("NORMAL DATA Sending");

          client.publish(macs, copy);
          client.loop();
          timer_counter_in_deep_sleep = 0;
        }
      }

      else
      {
        Serial.println("wifi DISconnected");
        wifi_rssi_sig();
        // esp_wifi_start();
        // measure_battery();
        //  vTaskDelay(1000);
        send_str.MAC = WiFi.macAddress();
        DateTime time = rtc.now();
        formattedDate = (time.timestamp(DateTime::TIMESTAMP_DATE));
        timeStamp = (time.timestamp(DateTime::TIMESTAMP_TIME));
        send_str.unix_time = time.unixtime();
        // send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + final_humidity + ",temperature=" + t + ",wifi=" + send_str.avg_temp_sig + ",battery=" + send_str.Bpercent + ",software_ver=" + FirmwareVer + ",alert_temp=" + send_str.Alert_temp + +",alert_humidity=" + send_str.Alert_humi + "\r\n";

        send_str.sensorData = "mac=" + send_str.MAC + ",time=" + send_str.unix_time + ",humidity=" + final_humidity + ",temperature=" + t + ",wifi=" + rtc_rssi_signal + ",battery=" + send_battery_value + "\r\n";

        // Serial.println(sensorData);
        vTaskDelay(100);
        str_lendata = send_str.sensorData.length() + 1;
        str_lenmac = send_str.MAC.length() + 1;

        char macs[str_lenmac];
        char copy[str_lendata];

        send_str.MAC.toCharArray(macs, str_lenmac);             //// find length
        send_str.sensorData.toCharArray(copy, str_lendata);     //// mac to char
        send_str.sensorData.toCharArray(str_data, str_lendata); //// mac to char

        Serial.println(copy);
        vTaskDelay(100);
      }

      vTaskDelay(50);
      if (Mqtt_flag1 == true)
      {
        Serial.println("RabbitMQ connected");
        // readFile(SD, "/smart sensing/hello.txt");

        // Serial.println("NORMAL DATA Sending");

        // client.publish(macs, copy);
        // client.loop();
        Mqtt_flag1 = false;
      }
      else
      {

        Serial.println("not connected to Rabbit mq");
        Serial.println("write to sd card");
        Serial.print("data  == ");
        Serial.println(str_data);
        appendFile(SD, "/smart sensing/hello.txt", str_data);
        chnage_temp_flag = false;
        delay(10);
      }
      chnage_temp_flag = false;
      chnage_humidity_flag = false;
      timer_1_hour_flag = false;
      force_send_data_flag = false;
    }

    // sht31_dsclock_Sleep(0x44, 0x68);

    ++portal_check;

    if (sett.sleep_time_activate == 1)
    {
      // esp_wifi_stop();
      client.disconnect();
      espClient.flush();
      WiFi.disconnect();
      esp_wifi_stop();
      delay(300);

      esp_sleep_enable_timer_wakeup(sett.sleep_time * uS_TO_S_FACTOR);

      Serial.println("Setup ESP32 to sleep for every " + String(sett.sleep_time) +
                     " Seconds");

      Serial.println("Going to sleep now");
      Serial.flush();
      esp_deep_sleep_start();
    }
    else
    {
      WiFi.disconnect();
      // Serial.println("No sleep");
      // lcd.write(1, 0b00000000);
      // lcd.write(2, 0b00000000);
    }
    vTaskDelay(1000);
  }
}

void setup()
{

  Serial.begin(115200);
  init_sensor_and_lcd();
  EEPROM.begin(512);
  rtc.begin();

  // Serial.print("Read previous temp from EEprom --> upper limit  ");
  // sett.upper_temp = EEPROM.read(0);
  // Serial.println(sett.upper_temp); // print

  // Serial.print("Read previous temp from EEprom --> lower limit  ");
  // sett.lower_temp = EEPROM.read(15);
  // Serial.println(sett.lower_temp); // print

  // Serial.print("Read previous humi from EEprom --> upper limit  ");
  // sett.upper_humi = EEPROM.read(25);
  // Serial.println(sett.upper_humi); /// print

  // Serial.print("Read previous humi from EEprom --> lower limit");
  // sett.lower_humi = EEPROM.read(35);
  // Serial.println(sett.lower_humi); /// print

  Serial.print("Read sleep time from EEprom in secs  ");
  sett.sleep_time = EEPROM.read(45);
  Serial.println(sett.sleep_time); /// print

  Serial.print("Read sleep time aqctivation ");
  sett.sleep_time_activate = EEPROM.read(55);
  Serial.println(sett.sleep_time_activate); /// print

  Serial.print("forcefully send variable= ");
  sett.force_send = EEPROM.read(65);
  Serial.print("EEprom Read done");

  // rtc.adjust(DateTime(1642404337));

  /////////////////////////////////////////////////////

  //rtc.adjust(DateTime(1643107623)); //put unix time to set the time zone
  if (portal_check == 0)
  {

    init_wifi_portal();
    Serial.println("portal set at first time");
  }
  else
  {

    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  }
  Serial.print("Active firmware version:");
  Serial.println(FirmwareVer);
  SD.begin(5);
  lcd.write(1, 0b00000000);
  lcd.write(2, 0b00000000);

  if (sett.force_send == 1)
  {

    force_send_data_flag = true;
  }
  else
  {
    force_send_data_flag = false;
    force_to_send_data();
  }
  // delay(1000);
  //  wifi_rssi_sig();
  //  rssi = WiFi.RSSI();
  //  Serial.println(rssi);

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // pinMode(lcd_background_pin, OUTPUT);    // sets the digital pin 13 as output
  // digitalWrite(lcd_background_pin, HIGH); // sets the digital pin 13 on

  // MAC = WiFi.macAddress();

  if( sett.active_set_time == 1)
  {
  int set_time;
   set_time =  sett.Time_display.toInt();
      rtc.adjust(DateTime(set_time)); //put unix time to set the time zone
Serial.println("Time set");
  }
  else{

    Serial.print("Previous time set/ to new time get");
  }


  send_str.MAC = WiFi.macAddress();
  client.setServer(mqtt_server, 1883);
  delay(1);
  createDir(SD, "/smart sensing");

  //  Serial.print("upper limit temp =  ");
  //  Serial.println(sett.upper_temp);
  //  Serial.print("lower upper limit temp = ");
  //  Serial.println(sett.lower_temp);

  ////start timer
  // timer = timerBegin(1, 80, true);
  // timerAttachInterrupt(timer, &onTimer, true);

  // timerAlarmWrite(timer, (TIMer_time * timer_uS_TO_S_FACTOR), true); // 1000000
  // timerAlarmWrite(timer, 000000, true); // 20000000
  // timerAlarmEnable(timer);
  //////////////
  // ++portal_check;
  // WiFi.disconnect();
  // WiFi.mode(WIFI_OFF);

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xBinarySemaphore = xSemaphoreCreateBinary();

  // mutex_v = xSemaphoreCreateMutex();
  // if (mutex_v == NULL) {
  // Serial.println("Mutex can not be created");
  //}

  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */
  delay(500);

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 1

  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
  delay(500);
  xSemaphoreGive(xBinarySemaphore);
}
void loop()
{
  delay(100);
}