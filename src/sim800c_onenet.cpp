#include "sim800c_onenet.h"
#define dbg

void sim800c::ssbegin(int baud) {
  mySerial.begin(baud);
}

boolean sim800c::resetModem() {
  uint8_t answer = 0;
  answer = sendATcommand("AT", "OK", 2000);
  if (answer == 1)sendATcommand("AT", "OK", 2000); // turn off the echo
  else if (answer == 0)
  {
    // waits for an answer from the module
    int trials = 0;
    while (answer == 0) {
      trials++;
      // Send AT every two seconds and wait for the answer
      answer = sendATcommand("AT", "OK", 1000);
      if (trials == 5) {
        Serial.println(F("GSM Start Fail"));
        //break;
        return false;
      }
    }
    sendATcommand("ATE0", "OK", 1000);// turn off the echo
  }
  else if (answer == 1)return true;
}

String sim800c::readServerResponse() {
  unsigned long nowMillis = millis();
  if (mySerial.available()) {
    while (char(Serial.read()) != 0x24) {
      if((millis() - nowMillis) > 2000) {
        Serial.println("NOT A COMMAND FROM REMOTE");
        //mySerial.flush();
        break;
      }
    }
    while (mySerial.available()) {
      String Msg = mySerial.readStringUntil('\r\n');
      Serial.print(Msg);
      shortMsg = Msg.substring(46);
      Serial.println("");
      return shortMsg;
      //Serial.print(char(mySerial.read()));
    }
  }
  return blank;
}

int8_t sim800c::sendATcommand2(char* ATcommand, char* expected_answer1, char* expected_answer2, unsigned int timeout) {

  uint8_t x = 0,  answer = 0;
  char response[100];
  unsigned long previous;

  memset(response, '\0', 100);    // Initialize the string
  delay(100);
  mySerial.flush();
  mySerial.println(ATcommand);    // Send the AT command
  //if(strstr(ATcommand, "AT+CIPSEND")!=NULL) mySerial.write(0x1A);

  //#ifdef dbg
  //  Serial.println(ATcommand);    // Send the AT command
  //#endif

  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {
    // if there are data in the UART input buffer, reads it and checks for the asnwer
    if (mySerial.available() != 0) {
      response[x] = mySerial.read();
      x++;
      // check if the desired answer 1  is in the response of the module

      if (strstr(response, expected_answer1) != NULL)
      {
        answer = 1;
        while (Serial.available()) {
          response[x] = mySerial.read();
          x++;
        }
      }
      // check if the desired answer 2 is in the response of the module
      else if (strstr(response, expected_answer2) != NULL)
      {
        answer = 2;
        while (Serial.available()) {
          response[x] = mySerial.read();
          x++;
        }
      }
    }
  } while ((answer == 0) && ((millis() - previous) < timeout)); // Waits for the asnwer with time out
#ifdef dbg
  Serial.println(response);
#endif
  return answer;
}
/////////////////////


int8_t sim800c::sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout) {

  uint8_t x = 0,  answer = 0;
  char response[500];
  unsigned long previous;
  char* str;
  uint8_t index = 0;
  memset(response, '\0', 100);    // Initialize the string
  delay(100);
  while ( mySerial.available() > 0) mySerial.read();   // Clean the input buffer
  mySerial.println(ATcommand);    // Send the AT command
#ifdef dbg
  Serial.println(ATcommand);    // Send the AT command
#endif
  x = 0;
  previous = millis();

  // this loop waits for the answer
  do {
    if (mySerial.available() != 0) {
      // if there are data in the UART input buffer, reads it and checks for the asnwer
      response[x] = mySerial.read();
      //mySerial.print(response[x]);
      x++;
      // check if the desired answer  is in the response of the module
      if (strstr(response, expected_answer) != NULL)
      {
        answer = 1;
      }
    }
  }
  // Waits for the asnwer with time out
  while ((answer == 0) && ((millis() - previous) < timeout));

#ifdef dbg
  Serial.println(response);    // Send the AT command
#endif
  return answer;
}


int sim800c::initTCP() {

  resetModem();
  sendATcommand2("ATE0", "OK", "ERROR", 2000);
  //  sendATcommand2("ATE0", "OK", "ERROR", 2000);

  delay(2000);


  Serial.println(F("Connecting to the network..."));

  while ( sendATcommand2("AT+CIPSHUT", "OK", "ERROR", 1000) == 0 );
  delay(2000);

  if (sendATcommand2("AT+CSQ", "+CSQ:", "OK", 1000) == 1)     //

  {
    while (sendATcommand("AT+CGATT?", "+CGATT: 1", 3000) == 0 );
    delay(1000);

    // Sets the APN, user name and password
    if (sendATcommand2("AT+CSTT=\"CMNET\"", "OK",  "ERROR", 3000) == 1)
    {

      // Waits for status IP START
      if (sendATcommand("AT+CIPSTATUS", "START", 500)  == 0 )
        delay(3000);

      // Brings Up Wireless Connection
      if (sendATcommand2("AT+CIICR", "OK", "ERROR", 3000) == 1)
      {

        // Gets Local IP Address
        if (sendATcommand2("AT+CIFSR", ".", "ERROR", 3000) == 1)
        {
          // Waits for status IP STATUS
          while (sendATcommand("AT+CIPSTATUS", "IP STATUS", 500)  == 0 );
          delay(3000);
          Serial.println(F("Opening TCP"));
          snprintf(aux_str, sizeof(aux_str), "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", MQTTHost, MQTTPort);

          // Opens a TCP socket
          if (sendATcommand2(aux_str, "OK\r\n\r\nCONNECT", "CONNECT FAIL", 3000) == 1)
          {
            Serial.println(F("Connected"));
            return 1;
          }
          else
          {
            Serial.println(F("Error opening the connection"));
            Serial.println(F("UNABLE TO CONNECT TO SERVER "));
            return 0;
          }
        }
        else
        {
          Serial.println(F("ERROR GETTING IP ADDRESS "));
          return 0;
        }
      }
      else {
        Serial.println(F("Error setting the APN"));
        return 0;
      }
    }
    else
    {
      Serial.println(F("Error setting NET"));
      return 0;
    }

  }
}

int sim800c::MQTTConnect(const char *id, const char *user, const char *pass) {
  return MQTTConnect(id, user, pass, 0, 0, 0, 0, 1);
}

int sim800c::MQTTConnect(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession) {

  if (sendATcommand2("AT+CIPSEND", ">", "ERROR", 1000)) {
    int nextMsgId = 1;
    // Leave room in the buffer for header and variable length field
    uint16_t length = MQTT_MAX_HEADER_SIZE;
    unsigned int j;

    //#if MQTT_VERSION == MQTT_VERSION_3_1
    //uint8_t d[9] = {0x00, 0x06, 'M', 'Q', 'I', 's', 'd', 'p', 3};
    //#define MQTT_HEADER_VERSION_LENGTH 9
    //#elif MQTT_VERSION == MQTT_VERSION_3_1_1        //ONENET对应的是3.1.1版本，所以下面最后一个参数是4
    uint8_t d[7] = {0x00, 0x04, 'M', 'Q', 'T', 'T', 4};
#define MQTT_HEADER_VERSION_LENGTH 7
    //#endif
    for (j = 0; j < MQTT_HEADER_VERSION_LENGTH; j++) {
      buffer[length++] = d[j];
    }

    uint8_t v;
    if (willTopic) {
      v = 0x04 | (willQos << 3) | (willRetain << 5);
    } else {
      v = 0x00;
    }
    if (cleanSession) {
      v = v | 0x02;
    }

    if (user != NULL) {
      v = v | 0x80;

      if (pass != NULL) {
        v = v | (0x80 >> 1);
      }
    }

    buffer[length++] = v;

    buffer[length++] = ((MQTT_KEEPALIVE) >> 8);
    buffer[length++] = ((MQTT_KEEPALIVE) & 0xFF);

    CHECK_STRING_LENGTH(length, id);
    length = writeString(id, buffer, length);
    if (willTopic) {
      CHECK_STRING_LENGTH(length, willTopic);
      length = writeString(willTopic, buffer, length);
      CHECK_STRING_LENGTH(length, willMessage);
      length = writeString(willMessage, buffer, length);
    }

    if (user != NULL) {
      CHECK_STRING_LENGTH(length, user);
      length = writeString(user, buffer, length);
      if (pass != NULL) {
        CHECK_STRING_LENGTH(length, pass);
        length = writeString(pass, buffer, length);
      }
    }

    writeCont(MQTTCONNECT, buffer, length - MQTT_MAX_HEADER_SIZE);
    mySerial.write(0x1A);
    if (sendATcommand("", "SEND OK", 1500)) {
      Serial.println(F("CONNECT PACKET SUCCESS"));
      return 1;
    }
    else Serial.println(F("CONNECT FAILED"));
    return 0;
  } else {
    snprintf(aux_str, sizeof(aux_str), "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", MQTTHost, MQTTPort);
    if (sendATcommand2(aux_str, "ERROR", "ALREADY", 3000) == 1 ) {
      Serial.println("TCP已连接模块……");
      return 0;
    } else {
      Serial.println("TCP失去连接，初始化模块……");
      do {
        s1 = initTCP();
      } while (s1 = 0);
    }
  }
}

boolean sim800c::writeCont(uint8_t header, uint8_t* buf, uint16_t length) {
  uint16_t rc;
  uint8_t hlen = buildHeader(header, buf, length);
  mySerial.write(buf + (MQTT_MAX_HEADER_SIZE - hlen), length + hlen);
  return (rc == hlen + length);
}

boolean sim800c::CHECK_STRING_LENGTH(int l, const char* s) {
  if (l + 2 + strlen(s) > MQTT_MAX_PACKET_SIZE)
  { //_client->stop();
    return false;
  }
}

uint16_t sim800c::writeString(const char* string, uint8_t* buf, uint16_t pos) {
  const char* idp = string;
  uint16_t i = 0;
  pos += 2;
  while (*idp) {
    buf[pos++] = *idp++;
    i++;
  }
  buf[pos - i - 2] = (i >> 8);
  buf[pos - i - 1] = (i & 0xFF);
  return pos;
}

uint8_t sim800c::buildHeader(uint8_t header, uint8_t* buf, uint16_t length) {
  uint8_t lenBuf[4];
  uint8_t llen = 0;
  uint8_t digit;
  uint8_t pos = 0;
  uint16_t len = length;
  do {
    digit = len % 128;
    len = len / 128;
    if (len > 0) {
      digit |= 0x80;
    }
    lenBuf[pos++] = digit;
    llen++;
  } while (len > 0);

  buf[4 - llen] = header;
  for (int i = 0; i < llen; i++) {
    buf[MQTT_MAX_HEADER_SIZE - llen + i] = lenBuf[i];
  }
  return llen + 1; // Full header size is variable length bit plus the 1-byte fixed header
}

void sim800c::public_data(char* tmp) {
  //snprintf(tmp,sizeof(tmp),"{\"light\":%d}",light_flag); //snprintf(tmp,sizeof(tmp),"{\"数据流ID1\":%d ,\"数据流ID2\":%d}",val_1,val_2);
  uint16_t streamLen = strlen(tmp);
  Serial.println(tmp);
  d[0] = '\x03';
  d[1] = (streamLen >> 8);
  d[2] = (streamLen & 0xFF);
  snprintf(msg, sizeof(msg), "%c%c%c%s", d[0], d[1], d[2], tmp);
  if (uploadMsg("$dp", (uint8_t*)msg, streamLen + 3, false)) {
    Serial.println("UPLOAD SUCCESS");
  } else {
    Serial.println("UPLOAD FAILED");
    failtimes++;
    if (failtimes >= 5) {
      failtimes = 0;
      do {
        s1 = initTCP();
      } while (s1 = 0);
    }
  }
}

boolean sim800c::uploadMsg(const char* topic, const uint8_t* payload, unsigned int plength, boolean retained) {
  if (MQTT_MAX_PACKET_SIZE < MQTT_MAX_HEADER_SIZE + 2 + strlen(topic) + plength) {// Too long
    return false;
  }
  // Leave room in the buffer for header and variable length field
  uint16_t length = MQTT_MAX_HEADER_SIZE;
  length = writeString(topic, buffer, length);
  uint16_t i;
  for (i = 0; i < plength; i++) {
    buffer[length++] = payload[i];
  }
  uint8_t header = MQTTPUBLISH;
  if (retained) {
    header |= 1;
  }
  if (sendATcommand2("AT+CIPSEND", ">", "ERROR", 1000)) {
    writeCont(MQTTPUBLISH, buffer, length - MQTT_MAX_HEADER_SIZE);
    mySerial.write(0x1A);
    if (sendATcommand("", "SEND OK", 1500)) {
      Serial.println("UPLOAD...");
      return true;
    } else return false;
  } else return false;
}
