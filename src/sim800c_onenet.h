#ifndef sim800c_onenet_h
#define sim800c_onenet_h

#include "Arduino.h"
#include <SoftwareSerial.h>

#define MQTT_MAX_HEADER_SIZE 5
#define MQTT_MAX_PACKET_SIZE 1024
#define MQTT_KEEPALIVE 90
#define MQTTCONNECT     1 << 4  // Client request to connect to Server
#define MQTTPUBLISH     3 << 4  // Publish message
#define MQTTPINGREQ     12 << 4 // PING Request

class sim800c {
  SoftwareSerial mySerial;

  const char * MQTTHost = "183.230.40.39";
  const char * MQTTPort = "6002";
  uint8_t buffer[MQTT_MAX_PACKET_SIZE];
  
  char aux_str[100];
  char msg[50];
  char d[3];
  String blank = "?";
  String shortMsg;
  int failtimes;
  int s1;

  public:
  sim800c(const int RX_PIN = 16, const int TX_PIN = 17):mySerial(RX_PIN, TX_PIN){}
  void ssbegin(int baud);
  String readServerResponse();
  int initTCP();
  int MQTTConnect(const char *id, const char *user, const char *pass);
  int MQTTConnect(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession);
  void public_data(char* tmp);
  boolean resetModem();
  int8_t sendATcommand2(char* ATcommand, char* expected_answer1, char* expected_answer2, unsigned int timeout);
  int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout);
  boolean writeCont(uint8_t header, uint8_t* buf, uint16_t length);
  boolean CHECK_STRING_LENGTH(int l, const char* s);
  uint16_t writeString(const char* string, uint8_t* buf, uint16_t pos);
  uint8_t buildHeader(uint8_t header, uint8_t* buf, uint16_t length);
  boolean uploadMsg(const char* topic, const uint8_t* payload, unsigned int plength, boolean retained);

};

#endif
