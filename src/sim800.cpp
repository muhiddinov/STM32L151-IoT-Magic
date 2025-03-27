#include "HardwareSerial.h"
#include "sim800.h"
#include <Arduino.h>

extern HardwareSerial SerialMON;
extern HardwareSerial SerialSIM;

void SIM800::sleep() {
  _responseOK("AT+CFUN=4");
  _internet = false;
}

void SIM800::wakeup() {
  _responseOK("AT");
  _responseOK("AT+CFUN=1");
  _responseOK("AT");
}

uint8_t SIM800::begin(uint32_t baudrate) {
  SerialMON.end();
  SerialSIM.end();
  SerialSIM.begin(baudrate);
  SerialMON.begin(baudrate);
  _responseOK("AT+CFUN=1");
  _responseOK("AT");
  _responseOK("ATE0");
  _responseOK("AT+SAPBR=0,1");
  _internet = false;
  _responseOK("AT+SAPBR=3,1,\"APN\",\"DEFAULT\"");
  _responseOK("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  // SIM800::getIMEI();
  // SerialMON.println(F("SIM800 is ready"));
  return _result_code;
}

uint8_t SIM800::checkPin() {
  SerialSIM.println("AT+CPIN?");
  SerialSIM.flush();
  uint32_t time_out = millis();
  while (1) {
    if (SerialSIM.available()) {
      _readData = SerialSIM.readStringUntil('\n');
      if (_readData.indexOf("+CPIN: READY") >= 0) {
        _result_code = errors(READY);
        break;
      } else if (_readData.indexOf("+CPIN: NOT INSERT") >= 0) {
        _result_code = errors(NOT_INSERT);
      }
    }
    if (millis() - time_out >= 2000) {
      _result_code = errors(timeout);
      break;
    }
  }
  return _result_code;
}

uint8_t SIM800::getRSSI() {
  SerialSIM.println("AT+CSQ");
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = SerialSIM.readStringUntil('\n');
    if (_readData.indexOf("+CSQ") >= 0) {
      int k = 0;
      sscanf(_readData.c_str(), "+CSQ: %d,%d", &_rssi, &k);
    }
  }
  return _rssi / 7 + 1;
}

String SIM800::getIMEI() {
  SerialSIM.println("AT+GSN");
  SerialSIM.flush();
  uint32_t resp_time = millis();
  while (1) {
    if (SerialSIM.available()) {
      _readData = SerialSIM.readStringUntil('\n');
      if (_readData.length() >= 7) {
        _readData.remove(_readData.length() - 1);
        _imei = _readData;
        SerialMON.print("Device ID: ");
        SerialMON.println(_imei);
        SerialMON.flush();
      }
      if (_readData.indexOf("OK") >= 0) {
        _result_code = errors(ok);
        break;
      }
    }
    if (millis() - resp_time > 2000) {
      _result_code = errors(timeout);
      break;
    }
  }
  return _imei;
}

uint16_t SIM800::_responseOK(const char* cmd) {
  SerialSIM.println(cmd);
  SerialSIM.flush();
  SerialMON.println(cmd);
  SerialMON.flush();
  delay(500);
  uint32_t time_out = millis();
  while (1) {
    if (SerialSIM.available()) {
      _readData = SerialSIM.readStringUntil('\n');
      SerialMON.println(_readData);
      SerialMON.flush();
      if (_readData.indexOf("OK") >= 0 || _readData.indexOf("ERROR") >= 0) {
        _result_code = errors(ok);
        break;
      }
    }
    if (millis() - time_out >= 2000) {
      _result_code = errors(timeout);
      break;
    }
  }
  return _result_code;
}

uint16_t SIM800::_responseGET(const char* cmd) {
  SerialSIM.println(cmd);
  SerialSIM.flush();
  SerialMON.println(cmd);
  SerialMON.flush();
  uint32_t time_out = millis();
  while (1) {
    if (SerialSIM.available()) {
      _readData = SerialSIM.readStringUntil('\n');
      SerialMON.println(_readData);
      SerialMON.flush();
      if (_readData.indexOf("+HTTPACTION") >= 0) {
        int method = 0, datalen = 0, resp = 0;
        sscanf(_readData.c_str(), "+HTTPACTION: %d,%d,%d", &method, &resp, &datalen);
        _responseCode = resp;
        break;
      }
    }
    if (millis() - time_out >= 15000) {
      _responseCode = 408;
      break;
    }
  }
  return _responseCode;
}

uint16_t SIM800::httpGet(const char* url) {
  SerialMON.print("URL:");
  SerialMON.println(url);
  SerialMON.flush();
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=0");
  _responseOK("AT+HTTPPARA=\"CID\",1");
  SerialSIM.print("AT+HTTPPARA=\"URL\",\"");
  SerialSIM.flush();
  SerialSIM.print(url);
  SerialSIM.flush();
  SerialSIM.println("\"");
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (SerialSIM.available()) {
      String str = SerialSIM.readStringUntil('\n');
      if (str.indexOf("OK") || str.indexOf("ERROR")) {
        break;
      }
    }
    if (millis() - timeout >= 1000) {
      break;
    }
  }
  uint16_t res = _responseGET("AT+HTTPACTION=0");
  _responseOK("AT+HTTPTERM");
  return res;
}

uint16_t SIM800::httpsGet(const char* url) {
//  SerialMON.print("URL:");
//  SerialMON.println(url);
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=1");
  _responseOK("AT+HTTPPARA=\"CID\",1");
  SerialSIM.print("AT+HTTPPARA=\"URL\",\"");
  SerialSIM.print(url);
  SerialSIM.println("\"");
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (SerialSIM.available()) {
      String str = SerialSIM.readStringUntil('\n');
      if (str.indexOf("OK") || str.indexOf("ERROR")) {
        break;
      }
    }
    if (millis() - timeout >= 1000) {
      break;
    }
  }
  uint16_t res = _responseGET("AT+HTTPACTION=0");
  _responseOK("AT+HTTPTERM");
  return res;
}

void SIM800::setAPN(const char* apn) {
  char url_request[100];
  sprintf(url_request, "AT+SAPBR=3,1,\"APN\",\"%s\"", apn);
  _responseOK("AT+SAPBR=0,1");
  _responseOK(url_request);
  _responseOK("AT+SAPBR=1,1");
}


void SIM800::openBearer() {
  _responseOK("AT+SAPBR=1,1");
  _internet = true;
}

void SIM800::closeBearer() {
  _responseOK("AT+SAPBR=0,1");
  _internet = false;
}

bool SIM800::getInternetStatus() {
  return _internet;
}

String SIM800::getIPAddress() {
  SerialSIM.println(F("AT+SAPBR=2,1"));
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) {
      break;
    }
    _readData = SerialSIM.readStringUntil('\n');
    if (_readData.indexOf("+SAPBR:") >= 0) {
      int k = 0, i = 0;
      sscanf(_readData.c_str(), "+SAPBR: %d,%d,\"%s\"", &k, &i, _wlan_ip_addr);
      _wlan_ip_addr[strlen(_wlan_ip_addr) - 1] = ' ';
      if (i == 1) _internet = true;
      else _internet = false;
    }
  }
  return String(_wlan_ip_addr);
}

uint16_t SIM800::getBattVoltage() {
  SerialSIM.println("AT+CBC");
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = SerialSIM.readStringUntil('\n');
    if (_readData.indexOf("+CBC") >= 0) {
      int k = sscanf(_readData.c_str(), "+CBC: 0,%d,%d", &_batt_percent, &_batt_voltage);
    }
  }
  return _batt_voltage;
}

uint8_t SIM800::getBattPercent() {
  SerialSIM.println("AT+CBC");
  SerialSIM.flush();
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = SerialSIM.readStringUntil('\n');
    if (_readData.indexOf("+CBC") >= 0) {
      int k = sscanf(_readData.c_str(), "+CBC: 0,%d,%d", &_batt_percent, &_batt_voltage);
    }
  }
  return _batt_percent;
}
