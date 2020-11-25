/*
* Ellias K Stuart
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>
//#include <avr/sleep.h>
//#include <avr/power.h>
#include <IRremote.h>
#include "I2Cdev.h"
//#include <SCoop.h>
#include <Metro.h>
#include "ESP8266.h"
#include "sim800c_onenet.h"

//#define IRDEBUG
#define DEBUG
//#define INTERRUPT_ENABLED
//#define ACCELGYRO_SERIAL_OUTPUT
//#define ACCIDENT_TEST
#define GPS_SERIAL_OUTPUT

#define pinInterrupt 2
#define lcdBackLight 28

#define GPS Serial1
#define SIM Serial2
#define ESPWIFI ESPSOFT

// defineTask(dataFetch);
// defineTask(dataUpload);
// defineTask(debugOutput);

Metro dataUpdate      = Metro(5000);
Metro dataFetch       = Metro(1000);
Metro accidentMonitor = Metro(100);

SoftwareSerial ESPSOFT(13,12);
MPU6050 accelgyro;
TinyGPSPlus gpsData;
ESP8266 WLAN(ESPSOFT);

const int rs = 23, en = 22, d4 = 27, d5 = 26, d6 = 25, d7 = 24;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

#define SSID "ELLIAS"
#define PASSWORD "Akimihomura!"
#define TEL_NUM 15724575401
#define HOST_NAME "api.heclouds.com"
#define HOST_PORT (80)
#define DEVICE_ID "644250210"
const String APIKey = "fhAS54e5X8HL5wcaB6ZW74oA3vo=";

void getGpsData();
void dataUpd();
void accidentReport();
void accelgyroSetUp();
bool rotateCheck();
int lcd_rm_encode(long long);
void nowPosiModify(long long);
void gotoSleep(void);
bool check_motion();

#define abs(x) ((x)<0? (-x):(x))

double Lati, Logi, Alti, Skmph, Smps, Acce, timeDelta;
int Year, Month, Day, Hour, Minute, Second, Timer;
long long interTimer;

char sout[101];

void setup() {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);

  Serial.begin(9600);
  GPS.begin(9600);
  ESPWIFI.begin(115200);
  SIM.begin(19200);

  lcd.print("[#");
  lcd.setCursor(10, 1);
  lcd.print("]");
  lcd.setCursor(1, 1);

  delay(3000);

  pinMode(pinInterrupt, INPUT);
  pinMode(lcdBackLight, OUTPUT);

  digitalWrite(lcdBackLight, HIGH);

  SIM.println("AT");

  lcd.print("#");

  Serial.print("setup begin\r\n");
  Serial.print("FW Version: ");
  Serial.println(WLAN.getVersion().c_str());

  lcd.print("#");

  if (WLAN.setOprToStation()) {
    Serial.print("to station ok\r\n");
  } else {
    Serial.print("to station err\r\n");
  } lcd.print("#");
  if (WLAN.joinAP(SSID, PASSWORD)) {
    Serial.print("Join AP success\r\n");
    Serial.print("IP: ");
    Serial.println(WLAN.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  } lcd.print("#");
  ESPWIFI.println("AT+UART_CUR=9600,8,1,0,0");
  ESPWIFI.begin(9600);

  lcd.print("#");

  interTimer = Timer = millis(); lcd.print("#");

  accelgyroSetUp(); lcd.print("#");

  irrecv.enableIRIn(); lcd.print("#");

  #ifdef ACCIDENT_TEST
  accidentReport ();
  #endif

  Serial.println("setup end\r\n"); lcd.print("#] OK.");

}

void loop() {

  #ifdef IRDEBUG

  if (irrecv.decode(&results)) {
    lcd.clear(); lcd.setCursor(0, 0);
    Serial.println(results.value, HEX);
    lcd.print(results.value, HEX);
    irrecv.resume();
  }

  #endif

  //Serial.println("Loop Runtime");

  while (GPS.available()) {
    if (gpsData.encode(GPS.read())) {
      
      if (dataFetch.check())  getGpsData();
      if (dataUpdate.check()) dataUpd();
      if (accidentMonitor.check()) {
        if (Skmph>=25.0 && rotateCheck() && abs(Acce)>=20.0) accidentReport();
      }
      
      #ifdef DEBUG
      register bool flag=irrecv.decode(&results);
      nowPosiModify(results.value);
      irrecv.resume(); if (flag) interTimer=millis();
      #endif
    }
  }

  #ifdef INTERRUPT_ENABLED
  if (millis()-interTimer>=10000 && check_motion()) gotoSleep();
  #endif
}

void dataUpd () {
  if (WLAN.createTCP(HOST_NAME, HOST_PORT)) {
      Serial.println("create tcp ok\r\n");
      char buf[10];
      String jsonToSend = "{\"Logitude\":";
      dtostrf(Logi, 1, 6, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += ",\"Latitude\":";
      dtostrf(Lati, 1, 6, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += ",\"Altitude\":";
      dtostrf(Alti, 1, 6, buf);
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
      // Serial.println("send success");
      // if (WLAN.releaseTCP()) {
      //   Serial.print("release tcp ok\r\n");
      // } else {
      //   Serial.print("release tcp err\r\n");
      // }
      postArray = NULL;
    } else {
      Serial.print("create tcp err\r\n");
    }
}

void getGpsData () {
  Year   = gpsData.date.year();
  Month  = gpsData.date.month();
  Day    = gpsData.date.day();
  Hour   = gpsData.time.hour();
  Minute = gpsData.time.minute();
  Second = gpsData.time.second();

  #ifdef GPS_SERIAL_OUTPUT

  sprintf(sout, "Date: %d/%d/%d %d:%d:%d\n", Year, Month, Day, Hour, Minute, Second);
  //Serial.print (sout);
  
  #endif

  Lati = gpsData.location.lat();
  Logi = gpsData.location.lng();
  Alti = gpsData.altitude.meters();
  
  #ifdef GPS_SERIAL_OUTPUT

  Serial.print  ("Latitude= ");
  Serial.print  (Lati, 6);
  Serial.print  (" Longitude= ");
  Serial.print  (Logi, 6);
  Serial.print  (" Altitude= ");
  Serial.println(Alti, 6);

  #endif

  Acce = (gpsData.speed.mps() - Smps) / (timeDelta=((double)(millis()-Timer)/1000.0));
  Timer = millis();

  Skmph = gpsData.speed.kmph();
  Smps  = gpsData.speed.mps();

  #ifdef GPS_SERIAL_OUTPUT

  Serial.print  ("Speed: ");
  Serial.print  (Skmph, 2);
  Serial.print  (" Acce: ");
  Serial.println(Acce, 10);

  #endif

  //delay(500);
}

void accidentReport () {
  Serial.println ("Accident Report Trigged.");
  SIM.begin(115200);
  SIM.println("AT\r"); delay(1000);
  SIM.println("AT+CMGF=1\r"); delay(1000);
  //SIM.println("AT+CSCA=\"+8613800100500\"\r"); delay(1000);
  SIM.println("AT+CMGS=\"+8613384009298\"\r"); delay(1000);
  SIM.print("test.\r\n"); delay(1000); SIM.write(0x1A);
  delay (10000);
}

// Kalman

unsigned long now, lastTime = 0;
double dt;

int16_t ax, ay, az, gx, gy, gz;
double aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;
long axo = 0, ayo = 0, azo = 0, gxo = 0, gyo = 0, gzo = 0;

double pi = 3.1415926, AcceRatio = 16384.0, GyroRatio = 131.0;

long long n_sample = 8;
double aaxs[8]={0}, aays[8]={0}, aazs[8]={0};
long long aax_sum, aay_sum,aaz_sum;

double a_x[10]={0}, a_y[10]={0}, a_z[10]={0}, g_x[10]={0}, g_y[10]={0}, g_z[10]={0};
double Px=1, Rx, Kx, Sx, Vx, Qx, Py=1, Ry, Ky, Sy, Vy, Qy, Pz=1, Rz, Kz, Sz, Vz, Qz;

void accelgyroSetUp () {
  Wire.begin(); accelgyro.initialize();
  unsigned short times = 200;
  for (register int i=0; i^times; ++ i) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axo += ax, ayo += ay, azo += az, gxo += gx, gyo += gy, gzo += gz;
  } axo /= times, ayo /= times, azo /= times, gxo /= times, gyo /= times, gzo /= times;
}

bool rotateCheck () {
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

  #ifdef ACCELGYRO_SERIAL_OUTPUT

  Serial.print(agx); Serial.print(","); Serial.print(agy); Serial.print(","); Serial.print(agz);
  Serial.println();

  #endif

  return false;
}

int ret, rettmp=-1;
bool flag=true;
const int tot_sta=5;

void nowPosiModify (long long res) {
  rettmp=lcd_rm_encode(res);
  if (rettmp != -1) {
    if (rettmp>1000 && rettmp<=1100) {
      if (true) {
        flag=false;
        switch (rettmp) {
          case 1001: {ret = (ret+1)%tot_sta; break;}
          case 1002: {ret = --ret<0? ret+tot_sta:ret; break;}
        }
      }
    } else ret=rettmp, flag=true;
  }

  digitalWrite(lcdBackLight, HIGH);

  switch (ret) {
    case 0: {
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print(Year); lcd.print("/");
      if (Month<10) lcd.print("0"); lcd.print(Month); lcd.print("/");
      if (Day  <10) lcd.print("0"); lcd.print(Day);
      lcd.setCursor(0, 1);
      if (Hour<10)   lcd.print("0"); lcd.print(Hour); lcd.print(":");
      if (Minute<10) lcd.print("0"); lcd.print(Minute); lcd.print(":");
      if (Second<10) lcd.print("0"); lcd.print(Second);
      break;
    } case 1: {
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print("Logi: "); lcd.print(Logi);
      lcd.setCursor(0, 1);
      lcd.print("Lati: "); lcd.print(Lati);
      break;
    } case 2: {
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print ("Speed(Kmph): ");
      lcd.setCursor(0, 1); lcd.print(Skmph);
      break;
    } case 3: {
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print ("Acceleration:");
      lcd.setCursor(0, 1); lcd.print(Acce);
      break;
    } case 4: {
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print("x:"); lcd.print(agx);
      lcd.print(" y:"); lcd.print(agy);
      lcd.setCursor(0, 1);
      lcd.print("z:"); lcd.print(agz);
      break;
    }
  } rettmp = -1;

  #ifdef DEBUG
  //Serial.print("ret: ");
  //Serial.print(ret);
  //Serial.print(" tettmp: ");
  //Serial.println(rettmp);
  #endif
}

int lcd_rm_encode (long long res) {
  switch (res) {
    case 0xFF6897: return 0;
    //Show Time
    case 0xFF30CF: return 1;
    //Show Longitude & Latitude
    case 0xFF18E7: return 2;
    //Show Speed
    case 0xFF7A85: return 3;
    //Show Acceleration
    case 0xFF10EF: return 4;
    //Show Accelgyro
    case 0xFF38C7: return 5;
    case 0xFF5AA5: return 6;
    case 0xFF42BD: return 7;
    case 0xFF4AB5: return 8;
    case 0xFF52AD: return 9;
    case 0xFF22DD: return 1001;
    case 0xFF02FD: return 1002;
    default: return -1;
  }
}

// void interruptFunc (void) {
//   lcd.clear(); digitalWrite(lcdBackLight, HIGH);
//   lcd.print("A  W  A  K  E");
//   detachInterrupt(0);
// }

// void gotoSleep (void) {
//   lcd.clear(); lcd.setCursor(0, 0);
//   lcd.print("FALL ASLEEP."); digitalWrite(lcdBackLight, LOW);
//   attachInterrupt(digitalPinToInterrupt(2), interruptFunc, LOW);
//   delay(100); set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//   sleep_enable(); sleep_mode(); sleep_disable();
// }

double preagx, preagy, preagz;

bool check_motion() {
  register bool ret=abs(agx-preagx)<=5&&abs(agy-preagy)<=5&&abs(agz-preagz)<=5&&Skmph<=5;
  if (!ret) interTimer=millis(); preagx=agx; preagy=agy; preagz=agz; return ret;
}