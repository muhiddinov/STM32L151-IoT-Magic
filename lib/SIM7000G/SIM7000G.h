#ifndef SIM7000G_H
#define SIM7000G_H
#include <Arduino.h>

enum errors {
  ok = 0,
  timeout = 10,
  err_val = 20,
  respons = 30,
  null_val = 40,
  READY = 1,
  NOT_INSERT = 2,
};

class SIM7000G {
  public:
    // serial  — UART connected to SIM7000G module
    // debug   — optional stream for AT log output (e.g. &SerialMON), pass nullptr to disable
    SIM7000G(HardwareSerial& serial, Stream* debug = nullptr);

    uint8_t begin(uint32_t baudrate);
    uint8_t checkPin();
    uint16_t getBattVoltage();
    uint8_t getBattPercent();
    uint8_t getRSSI();
    String getIPAddress();
    void openBearer();
    void closeBearer();
    void setAPN(const char*);
    uint16_t httpGet(const char*);
    uint16_t httpsGet(const char*);
    String getIMEI();
    void sleep();
    void wakeup();
    bool getInternetStatus();

    bool enableGPS();
    void disableGPS();
    bool getGPS(float* lat, float* lon);
    uint32_t getUnixTime();

  private:
    HardwareSerial& _serial;
    Stream*         _debug;

    uint16_t _responseOK(const char*);
    uint16_t _responseGET(const char*);
    uint16_t _responseCode;
    String   _readData;
    String   _imei;
    uint8_t  _batt_percent;
    uint16_t _batt_voltage;
    uint8_t  _rssi;
    char     _wlan_ip_addr[16];
    bool     _internet;
    uint8_t  _result_code;

    inline void _log(const char* s)   { if (_debug) _debug->println(s); }
    inline void _log(const String& s) { if (_debug) _debug->println(s); }
};

#endif
