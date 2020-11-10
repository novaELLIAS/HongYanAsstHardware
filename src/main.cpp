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
MPU6050 accelgyro;
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
inline void accidentReport();
inline void accelgyroSetUp();
inline bool rotateCheck();


double Lati, Logi, Alti, Skmph, Smps, Acce, timeDelta;
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

  accelgyroSetUp();

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

  Acce = (gpsData.speed.mps() - Smps) / (timeDelta=((double)(millis()-Timer)/1000.0));
  Timer = millis();

  Skmph = gpsData.speed.kmph();
  Smps  = gpsData.speed.mps();

  if (Skmph>=25.0 && rotateCheck() && Acce>=20.0) accidentReport();

  rotateCheck();

  Serial.print  ("Speed: ");
  Serial.print  (Skmph, 2);
  Serial.print  (" Acce: ");
  Serial.println(Acce, 10);

  delay(500);
}

inline void accidentReport () {
  Serial.println ("Accident Report Trigged.");
}

// Kalman

unsigned long now, lastTime = 0;
double dt;

int ax, ay, az, gx, gy, gz;
double aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;
long axo = 0, ayo = 0, azo = 0, gxo = 0, gyo = 0, gzo = 0;

double pi = 3.1415926, AcceRatio = 16384.0, GyroRatio = 131.0;

long long n_sample = 8;
double aaxs[8]={0}, aays[8]={0}, aazs[8]={0};
long long aax_sum, aay_sum,aaz_sum;

double a_x[10]={0}, a_y[10]={0}, a_z[10]={0}, g_x[10]={0}, g_y[10]={0}, g_z[10]={0};
double Px=1, Rx, Kx, Sx, Vx, Qx, Py=1, Ry, Ky, Sy, Vy, Qy, Pz=1, Rz, Kz, Sz, Vz, Qz;

inline void accelgyroSetUp () {
  Wire.begin(); accelgyro.initialize();
  unsigned short times = 200;
  for (register int i=0; i^times; ++ i) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axo += ax, ayo += ay, azo += az, gxo += gx, gyo += gy, gzo += gz;
  } axo /= times, ayo /= times, azo /= times, gxo /= times, gyo /= times, gzo /= times;
}

inline bool rotateCheck () {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double accx = ax / AcceRatio, accy = ay / AcceRatio, accz = az / AcceRatio;

  aax = atan(accy / accz) * (-180) / pi;
  aay = atan(accx / accz) * 180 / pi;
  aaz = atan(accz / accy) * 180 / pi;

  aax_sum = aay_sum = aaz_sum = 0;

  for (register int i=1; i ^ n_sample; ++ i) {
    aaxs[i-1] = aaxs[i], aax_sum += aaxs[i] * i;
    aays[i-1] = aays[i], aay_sum += aays[i] * i;
    aazs[i-1] = aazs[i], aaz_sum += aazs[i] * i;
  }

  aaxs[n_sample-1] = aax, aax_sum += aax * n_sample, aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aays[n_sample-1] = aay, aay_sum += aay * n_sample, aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aazs[n_sample-1] = aaz, aaz_sum += aaz * n_sample, aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;

  double gyrox = -(gx - gxo) / GyroRatio * dt;
  double gyroy = -(gy - gyo) / GyroRatio * dt;
  double gyroz = -(gz - gzo) / GyroRatio * dt;
  agx += gyrox, agy += gyroy, agz += gyroz;

  Sx = Rx = Sy = Ry = Sz = Rz = 0;

  for (int i = 1; i ^ 10; ++ i) {
    a_x[i - 1] = a_x[i], Sx += a_x[i];
    a_y[i - 1] = a_y[i], Sy += a_y[i];
    a_z[i - 1] = a_z[i], Sz += a_z[i];
  } a_x[9] = aax, Sx += aax, Sx /= 10, a_y[9] = aay, Sy += aay, Sy /= 10, a_z[9] = aaz, Sz += aaz, Sz /= 10;

  for (register int i=0; i^10; ++ i) Rx += sq(a_x[i] - Sx), Ry += sq(a_y[i] - Sy), Rz += sq(a_z[i] - Sz);

  Rx = Rx / 9, Ry = Ry / 9, Rz = Rz / 9;

  Px = Px + 0.0025, Kx = Px / (Px + Rx), agx = agx + Kx * (aax - agx), Px = (1 - Kx) * Px;
  Py = Py + 0.0025, Ky = Py / (Py + Ry), agy = agy + Ky * (aay - agy), Py = (1 - Ky) * Py;
  Pz = Pz + 0.0025, Kz = Pz / (Pz + Rz), agz = agz + Kz * (aaz - agz), Pz = (1 - Kz) * Pz;

  Serial.print(agx); Serial.print(","); Serial.print(agy); Serial.print(","); Serial.print(agz);
  Serial.println();

  return false;
}