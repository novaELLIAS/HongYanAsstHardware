/*
* Ellias K Stuart
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include "I2Cdev.h"
#include "ESP8266.h"

#define GPS Serial1
#define SIM Serial2
#define ESPWIFI ESPSOFT

SoftwareSerial ESPSOFT(13,12);
TinyGPSPlus gpsData;
ESP8266 WLAN(ESPSOFT);

#define SSID "ELLIAS"
#define PASSWORD "Akimihomura!"
#define HOST_NAME "api.heclouds.com"
#define HOST_PORT (80)
#define DEVICE_ID "644250210"
const String APIKey = "fhAS54e5X8HL5wcaB6ZW74oA3vo=";

inline void getGpsData();
inline void dataUpd();


double Lati, Logi, Alti, Skmph, Smps, Acce;
int Year, Month, Day, Hour, Minute, Second, Timer;

char sout[101];

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  ESPWIFI.begin(115200);
  SIM.begin(9600);
  SIM.println("AT+CMGF=1");

  Serial.print("setup begin\r\n");
  Serial.print("FW Version: ");
  Serial.println(WLAN.getVersion().c_str());
  if (WLAN.setOprToStation()) {
    Serial.print("to station ok\r\n");
  } else {
    Serial.print("to station err\r\n");
  }
  if (WLAN.joinAP(SSID, PASSWORD)) {
    Serial.print("Join AP success\r\n");
    Serial.print("IP: ");
    Serial.println(WLAN.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  }
  ESPWIFI.println("AT+UART_CUR=9600,8,1,0,0");
  ESPWIFI.begin(9600);

  Timer = millis();

  Serial.println("setup end\r\n");
}

void loop() {
  //Serial.print  ("GPS available? ");
  //Serial.println(GPS.available());
  while (GPS.available()) {
    if (gpsData.encode(GPS.read())) {
      getGpsData(); dataUpd();
    }
  }
}

inline void dataUpd () {
  if (WLAN.createTCP(HOST_NAME, HOST_PORT)) {
      Serial.print("create tcp ok\r\n");
      char buf[10];
      String jsonToSend = "{\"Logitude\":";
      dtostrf(Logi, 1, 2, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += ",\"Latitude\":";
      dtostrf(Lati, 1, 2, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += ",\"Speed\":";
      dtostrf(Skmph, 1, 2, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += "}";

      String postString = "POST /devices/";
      postString += DEVICE_ID;
      postString += "/datapoints?type=3 HTTP/1.1";
      postString += "\r\n";
      postString += "api-key:";
      postString += APIKey;
      postString += "\r\n";
      postString += "Host:api.heclouds.com\r\n";
      postString += "Connection:close\r\n";
      postString += "Content-Length:";
      postString += jsonToSend.length();
      postString += "\r\n";
      postString += "\r\n";
      postString += jsonToSend;
      postString += "\r\n";
      postString += "\r\n";
      postString += "\r\n";

      const char *postArray = postString.c_str();
      Serial.println(postArray);
      WLAN.send((const uint8_t *)postArray, strlen(postArray));
      Serial.println("send success");
      if (WLAN.releaseTCP()) {
        Serial.print("release tcp ok\r\n");
      } else {
        Serial.print("release tcp err\r\n");
      }
      postArray = NULL;
    } else {
      Serial.print("create tcp err\r\n");
    }
}

inline void getGpsData () {
  Year   = gpsData.date.year();
  Month  = gpsData.date.month();
  Day    = gpsData.date.day();
  Hour   = gpsData.time.hour();
  Minute = gpsData.time.minute();
  Second = gpsData.time.second();

  sprintf(sout, "Date: %d/%d/%d %d:%d:%d\n", Year, Month, Day, Hour, Minute, Second);
  Serial.print (sout);
  
  Lati = gpsData.location.lat();
  Logi = gpsData.location.lng();
  Alti = gpsData.altitude.meters();
  
  Serial.print  ("Latitude= ");
  Serial.print  (Lati, 6);
  Serial.print  (" Longitude= ");
  Serial.print  (Logi, 6);
  Serial.print  (" Altitude= ");
  Serial.println(Alti, 6);

  Acce = (gpsData.speed.mps() - Smps) / (double)(millis()-Timer);
  Timer = millis();

  Skmph = gpsData.speed.kmph();
  Smps  = gpsData.speed.mps();

  Serial.print  ("Speed: ");
  Serial.print  (Skmph, 2);
  Serial.print  (" Acce: ");
  Serial.print  (Acce, 2);
  Serial.println(" kph");

  delay(500);
}