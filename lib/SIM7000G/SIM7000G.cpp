#include "SIM7000G.h"
#include <Arduino.h>

// ─── Constructor ─────────────────────────────────────────────────

SIM7000G::SIM7000G(HardwareSerial& serial, Stream* debug)
  : _serial(serial), _debug(debug),
    _responseCode(0), _batt_percent(0), _batt_voltage(0),
    _rssi(0), _internet(false), _result_code(0) {
  memset(_wlan_ip_addr, 0, sizeof(_wlan_ip_addr));
}

// ─── Internal helpers ────────────────────────────────────────────

static uint32_t _toUnix(int yy, int mo, int dd, int hh, int mm, int ss) {
  const uint8_t dpm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  uint32_t days = 0;
  for (int y = 0; y < yy; y++) days += ((y % 4 == 0) ? 366 : 365);
  bool leap = (yy % 4 == 0);
  for (int m = 1; m < mo; m++) days += dpm[m-1] + (m == 2 && leap ? 1 : 0);
  days += dd - 1;
  return 946684800UL + days * 86400UL + (uint32_t)hh * 3600UL + mm * 60UL + ss;
}

// ─── Core ────────────────────────────────────────────────────────

uint16_t SIM7000G::_responseOK(const char* cmd) {
  while (_serial.available()) {_serial.read();};
  _serial.println(cmd);
  _serial.flush();
  _log(cmd);
  delay(300);
  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      _log(_readData);
      if (_readData.indexOf("OK") >= 0)    { _result_code = errors(ok);      return _result_code; }
      if (_readData.indexOf("ERROR") >= 0) { _result_code = errors(err_val); return _result_code; }
    }
  }
  _result_code = errors(timeout);
  return _result_code;
}

uint16_t SIM7000G::_responseGET(const char* cmd) {
  while (_serial.available()) {_serial.read();};
  _serial.println(cmd);
  _serial.flush();
  _log(cmd);
  uint32_t t = millis();
  while (millis() - t < 15000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      _log(_readData);
      if (_readData.indexOf("+HTTPACTION") >= 0) {
        int method = 0, datalen = 0, resp = 0;
        sscanf(_readData.c_str(), "+HTTPACTION: %d,%d,%d", &method, &resp, &datalen);
        _responseCode = resp;
        return _responseCode;
      }
    }
  }
  _responseCode = 408;
  return _responseCode;
}

// ─── Init ────────────────────────────────────────────────────────

uint8_t SIM7000G::begin(uint32_t baudrate) {
  _serial.end();
  _serial.begin(baudrate);
  delay(1000);

  // Send AT repeatedly until modem responds (auto-baud sync)
  bool alive = false;
  for (uint8_t i = 0; i < 10 && !alive; i++) {
    _serial.println("AT");
    _serial.flush();
    uint32_t t = millis();
    while (millis() - t < 1000) {
      if (_serial.available()) {
        String r = _serial.readStringUntil('\n');
        r.trim();
        _log(r);
        if (r.indexOf("OK") >= 0 || r.indexOf("AT") >= 0) { alive = true; break; }
      }
    }
    if (!alive) {
      if (_debug) { _debug->print("No resp, retry "); _debug->println(i + 1); }
      delay(500);
    }
  }

  if (!alive) {
    _log("MODEM NOT RESPONDING");
    _result_code = errors(timeout);
    return _result_code;
  }

  _responseOK("ATE0");
  _responseOK("AT+CFUN=1");
  _responseOK("AT+CNMP=13");
  _responseOK("AT+CLTS=1");
  _responseOK("AT+CSCLK=0");
  _responseOK("AT+SAPBR=0,1");
  _responseOK("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  _responseOK("AT+SAPBR=3,1,\"APN\",\"internet\"");
  _internet = false;
  return _result_code;
}

void SIM7000G::sleep() {
  _responseOK("AT+CFUN=4");
  _internet = false;
}

void SIM7000G::wakeup() {
  _responseOK("AT");
  _responseOK("AT+CFUN=1");
}

// ─── SIM & network ───────────────────────────────────────────────

uint8_t SIM7000G::checkPin() {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+CPIN?");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      if (_readData.indexOf("+CPIN: READY") >= 0)      { _result_code = errors(READY);      break; }
      if (_readData.indexOf("+CPIN: NOT INSERT") >= 0) { _result_code = errors(NOT_INSERT); break; }
    }
  }
  if (millis() - t >= 2000) _result_code = errors(timeout);
  return _result_code;
}

uint8_t SIM7000G::getRSSI() {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+CSQ");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      if (_readData.indexOf("+CSQ") >= 0) {
        int k = 0;
        sscanf(_readData.c_str(), "+CSQ: %hhu,%d", &_rssi, &k);
      }
    }
  }
  return _rssi / 7 + 1;
}

String SIM7000G::getIMEI() {
  // Bufer tozalash
  delay(5000);
  while (_serial.available()) { _serial.read(); }

  _imei = "";  // eski qiymatni tiklash
  _serial.println("AT+GSN");
  _serial.flush();

  uint32_t t = millis();
  while (millis() - t < 5000) {
    if (_serial.available()) {
      String line = _serial.readStringUntil('\n');
      line.trim();

      if (line.length() == 0) continue;           // bo'sh qator
      if (line.startsWith("AT")) continue;        // echo
      if (line == "OK" || line == "ERROR") continue; // javob tugadi

      // IMEI aniq 15 ta raqam bo'lishi kerak
      if (line.length() == 15) {
        bool allDigits = true;
        for (uint8_t i = 0; i < line.length(); i++) {
          if (!isDigit(line.charAt(i))) { allDigits = false;}
        }
        if (allDigits) {
          _imei = line;
          break;
        }
      }
    }
  }
  if (_debug) { _debug->print("IMEI: "); _debug->println(_imei); }
  return _imei;
}

uint32_t SIM7000G::getUnixTime() {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+CCLK?");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 3000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      if (_readData.indexOf("+CCLK:") >= 0) {
        int yy=0, mo=0, dd=0, hh=0, mm=0, ss=0, tz=0;
        sscanf(_readData.c_str(), "+CCLK: \"%d/%d/%d,%d:%d:%d+%d\"",
               &yy, &mo, &dd, &hh, &mm, &ss, &tz);
        if (yy > 0 && mo > 0) {
          // hh -= tz;
          return _toUnix(yy, mo, dd, hh, mm, ss);
        }
      }
    }
  }
  return 0;
}

// ─── Battery ─────────────────────────────────────────────────────

uint16_t SIM7000G::getBattVoltage() {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+CBC");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      if (_readData.indexOf("+CBC") >= 0)
        sscanf(_readData.c_str(), "+CBC: 0,%hhu,%hu", &_batt_percent, &_batt_voltage);
    }
  }
  return _batt_voltage;
}

uint8_t SIM7000G::getBattPercent() {
  getBattVoltage();
  return _batt_percent;
}

// ─── Bearer / internet ───────────────────────────────────────────

void SIM7000G::setAPN(const char* apn) {
  char cmd[80];
  _responseOK("AT+SAPBR=0,1");
  snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", apn);
  _responseOK(cmd);
}

void SIM7000G::openBearer() {
  _responseOK("AT+SAPBR=1,1");
  _internet = true;
}

void SIM7000G::closeBearer() {
  _responseOK("AT+SAPBR=0,1");
  _internet = false;
}

bool SIM7000G::getInternetStatus() {
  return _internet;
}

String SIM7000G::getIPAddress() {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+SAPBR=2,1");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 3000) {
    if (_serial.available()) {
      _readData = _serial.readStringUntil('\n');
      _readData.trim();
      if (_readData.indexOf("+SAPBR:") >= 0) {
        int k = 0, i = 0;
        sscanf(_readData.c_str(), "+SAPBR: %d,%d,\"%15[^\"]\"", &k, &i, _wlan_ip_addr);
        _internet = (i == 1);
      }
    }
  }
  return String(_wlan_ip_addr);
}

// ─── HTTP ────────────────────────────────────────────────────────

uint16_t SIM7000G::httpGet(const char* url) {
  if (_debug) { _debug->print("GET "); _debug->println(url); }
  _responseOK("AT+HTTPTERM");
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=0");
  _responseOK("AT+HTTPPARA=\"CID\",1");

  _serial.print("AT+HTTPPARA=\"URL\",\"");
  _serial.print(url);
  _serial.println("\"");
  _serial.flush();

  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      String s = _serial.readStringUntil('\n');
      s.trim();
      _log(s);
      if (s.indexOf("OK") >= 0 || s.indexOf("ERROR") >= 0) break;
    }
  }

  uint16_t res = _responseGET("AT+HTTPACTION=0");
  _responseOK("AT+HTTPTERM");
  if (_debug) { _debug->print("HTTP: "); _debug->println(res); }
  return res;
}

uint16_t SIM7000G::httpsGet(const char* url) {
  _responseOK("AT+HTTPTERM");
  _responseOK("AT+HTTPINIT");
  _responseOK("AT+HTTPSSL=1");
  _responseOK("AT+HTTPPARA=\"CID\",1");
  while (_serial.available()) {_serial.read();};
  _serial.print("AT+HTTPPARA=\"URL\",\"");
  _serial.print(url);
  _serial.println("\"");
  _serial.flush();

  uint32_t t = millis();
  while (millis() - t < 2000) {
    if (_serial.available()) {
      String s = _serial.readStringUntil('\n');
      s.trim();
      _log(s);
      if (s.indexOf("OK") >= 0 || s.indexOf("ERROR") >= 0) break;
    }
  }

  uint16_t res = _responseGET("AT+HTTPACTION=0");
  _responseOK("AT+HTTPTERM");
  return res;
}

// ─── GPS ─────────────────────────────────────────────────────────

bool SIM7000G::enableGPS() {
  _responseOK("AT+CGNSPWR=1");
  delay(1000);
  return (_result_code == errors(ok));
}

void SIM7000G::disableGPS() {
  _responseOK("AT+CGNSPWR=0");
}

bool SIM7000G::getGPS(float* lat, float* lon) {
  while (_serial.available()) {_serial.read();};
  _serial.println("AT+CGNSINF");
  _serial.flush();
  uint32_t t = millis();
  while (millis() - t < 3000) {
    if (!_serial.available()) continue;
    String line = _serial.readStringUntil('\n');
    line.trim();
    if (line.indexOf("+CGNSINF:") < 0) continue;

    // Tokenize by comma, collect first 5 fields
    int fi = 0;
    String fields[5];
    int start = line.indexOf(':') + 2;
    for (int i = start; i < (int)line.length() && fi < 5; i++) {
      if (line[i] == ',') { fi++; }
      else if (fi < 5)    { fields[fi] += line[i]; }
    }

    if (fields[0].toInt() == 0) return false;  // GNSS off
    if (fields[1].toInt() == 0) return false;  // no fix

    float la = fields[3].toFloat();
    float lo = fields[4].toFloat();
    if (la == 0.0f && lo == 0.0f) return false;

    *lat = la;
    *lon = lo;
    return true;
  }
  return false;
}
