#ifndef SIM800_H
#define SIM800_H
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

class SIM800 {
  public:
    uint8_t begin(uint32_t);
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

  private:
    uint16_t _responseOK(const char*);
    uint16_t _responseGET(const char*);
    uint16_t _responseCode;
    String _readData;
    String _imei;
    uint8_t _batt_percent;
    uint16_t _batt_voltage;
    uint8_t _rssi;
    char _wlan_ip_addr[15];
    String _coordinate;
    bool _internet;
    uint8_t _result_code;
};

#endif
