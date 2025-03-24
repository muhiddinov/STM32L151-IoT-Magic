#include "HardwareSerial.h"
#include "sim800.h"
#include <Arduino.h>

HardwareSerial Serial3(PB11, PB10);

#define SerialMon Serial3

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
  Serial1.begin(baudrate);
  SerialMon.begin(baudrate);
  uint32_t timeout = millis();
  _responseOK("AT+CFUN=1");
  _responseOK("AT");
  _responseOK("ATE0");
  _responseOK("AT+SAPBR=0,1");
  _internet = false;
  _responseOK("AT+SAPBR=3,1,\"APN\",\"INTERNET\"");
  _responseOK("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  SIM800::getIMEI();
  return _result_code;
}

uint8_t SIM800::checkPin() {
  Serial1.println("AT+CPIN?");
  uint32_t time_out = millis();
  while (1) {
    if (Serial1.available()) {
      _readData = Serial1.readStringUntil('\n');
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
  Serial1.println("AT+CSQ");
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = Serial1.readStringUntil('\n');
    if (_readData.indexOf("+CSQ") >= 0) {
      int k = 0;
      sscanf(_readData.c_str(), "+CSQ: %d,%d", &_rssi, &k);
    }
  }
  return _rssi / 7 + 1;
}

String SIM800::getIMEI() {
  Serial1.println("AT+GSN");
  uint32_t resp_time = millis();
  while (1) {
    if (Serial1.available()) {
      _readData = Serial1.readStringUntil('\n');
      if (_readData.length() >= 7) {
        _readData.remove(_readData.length() - 1);
        _imei = _readData;
        SerialMon.print("Device ID: ");
        SerialMon.println(_imei);
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
  Serial1.println(cmd);
  SerialMon.println(cmd);
  uint32_t time_out = millis();
  while (1) {
    if (Serial1.available()) {
      _readData = Serial1.readStringUntil('\n');
      SerialMon.println(_readData);
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
  Serial1.println(cmd);
  SerialMon.println(cmd);
  uint32_t time_out = millis();
  while (1) {
    if (Serial1.available()) {
      _readData = Serial1.readStringUntil('\n');
      SerialMon.println(_readData);
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
  SerialMon.print("URL:");
  SerialMon.println(url);
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=0");
  _responseOK("AT+HTTPPARA=\"CID\",1");
  Serial1.print("AT+HTTPPARA=\"URL\",\"");
  Serial1.print(url);
  Serial1.println("\"");
  uint32_t timeout = millis();
  while (1) {
    if (Serial1.available()) {
      String str = Serial1.readStringUntil('\n');
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
//  SerialMon.print("URL:");
//  SerialMon.println(url);
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=1");
  _responseOK("AT+HTTPPARA=\"CID\",1");
  Serial1.print("AT+HTTPPARA=\"URL\",\"");
  Serial1.print(url);
  Serial1.println("\"");
  uint32_t timeout = millis();
  while (1) {
    if (Serial1.available()) {
      String str = Serial1.readStringUntil('\n');
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
  Serial1.println(F("AT+SAPBR=2,1"));
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) {
      break;
    }
    _readData = Serial1.readStringUntil('\n');
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
  Serial1.println("AT+CBC");
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = Serial1.readStringUntil('\n');
    if (_readData.indexOf("+CBC") >= 0) {
      int k = sscanf(_readData.c_str(), "+CBC: 0,%d,%d", &_batt_percent, &_batt_voltage);
    }
  }
  return _batt_voltage;
}

uint8_t SIM800::getBattPercent() {
  Serial1.println("AT+CBC");
  uint32_t timeout = millis();
  while (1) {
    if (millis() - timeout >= 2000) break;
    _readData = Serial1.readStringUntil('\n');
    if (_readData.indexOf("+CBC") >= 0) {
      int k = sscanf(_readData.c_str(), "+CBC: 0,%d,%d", &_batt_percent, &_batt_voltage);
    }
  }
  return _batt_percent;
}
