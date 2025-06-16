#include <STM32LowPower.h>
#include <Arduino.h>
#include "sim800.h"

HardwareSerial SerialSIM(PB7, PB6);
HardwareSerial SerialMON(PA10, PA9);
HardwareSerial SerialUS(PA3, PA2);

SIM800 sim800;

#define GSM_PWR_PIN PB8
#define V33_PWR_PIN PA11
#define V42_PWR_PIN PA12
#define uS_TO_S 1000000
#define mS_TO_S 1000
#define SECONDS_IN_HOUR 3600 // seconds in an hour
#define TIME_SLEEP 1  // in hours

String coordinate = "40.141660,65.365109";
String url_server = "http://goldenx.uz/api/data.php?";
String imei = "";
int batt = 0;
int rssi_db = 0;
float distance = 0.0;
float temperature = 0.0;

// http://goldenx.uz/api/data.php?id=1234&location=123.124,63123.1332&water_level=200&water_volume=100&power=100&signal=4
String url_make(String id, 
                String location, 
                float water_level, 
                float water_volume, 
                int power = 100, 
                int rssi = 3) {

  return (url_server + "id=" + id + 
          "&location=" + coordinate + 
          "&water_level=" + String(water_level) + 
          "&water_volume=" + String(water_volume) + 
          "&power=" + String(power) + 
          "&signal=" + String(rssi));
}

void shutdown (uint32_t ms) {
  digitalWrite(V33_PWR_PIN, 1);
  digitalWrite(V42_PWR_PIN, 1);
  LowPower.shutdown(ms);
  NVIC_SystemReset();
}

uint16_t getDistance() {
  digitalWrite(V33_PWR_PIN, 0);
  delay(1000);
  uint16_t distance_mm = 0;
  uint8_t readReg[4];
  uint32_t cur_time = millis();
  while (millis() - cur_time < 5000) {
    if (SerialUS.available()) {
      SerialUS.readBytesUntil(0xFF, readReg, 3);
      distance_mm = (distance_mm + ((readReg[0] << 8) | readReg[1])) / 2;
    }
  }
  delay(1000);
  digitalWrite(V33_PWR_PIN, 1);
  return distance_mm;
}

void http_task () {
  digitalWrite(V42_PWR_PIN, 0);
  digitalWrite(GSM_PWR_PIN, 1);
  delay(2000);
  digitalWrite(GSM_PWR_PIN, 0);
  delay(5000);
  sim800.begin(9600);
  rssi_db = sim800.getRSSI();
  batt = sim800.getBattPercent();
  imei = sim800.getIMEI();
  sim800.openBearer();
  delay(2000);
  String ip = String(sim800.getIPAddress());
  SerialMON.println("WAN IP address: " + ip);
  SerialMON.flush();
  sim800.httpGet(url_make(imei, coordinate, distance, 0.0, batt, rssi_db).c_str());
  sim800.closeBearer();
  digitalWrite(V42_PWR_PIN, 1);
}

void setup() {
  pinMode(V33_PWR_PIN, OUTPUT);
  pinMode(V42_PWR_PIN, OUTPUT);
  pinMode(GSM_PWR_PIN, OUTPUT);
  digitalWrite(V33_PWR_PIN, 1);
  digitalWrite(V42_PWR_PIN, 1);
  delay(1000);
  // digitalWrite(V33_PWR_PIN, 0);
  // digitalWrite(V42_PWR_PIN, 0);
  digitalWrite(GSM_PWR_PIN, 1);
  delay(2000);
  digitalWrite(GSM_PWR_PIN, 0); 
  SerialUS.begin(9600);
  SerialMON.begin(9600);
  SerialSIM.begin(9600);
  SerialMON.println("Start");
  SerialMON.flush();
  distance = float(getDistance()) / 10.0; // distance in cm
  SerialMON.print("Distance: ");
  SerialMON.println(distance);
  // http_task();
  // shutdown(TIME_SLEEP * SECONDS_IN_HOUR * mS_TO_S);
}

void loop() {
  // SerialMON.println("Hello world!");
  // delay(1000);
  // if (SerialMON.available()) {
  //   SerialSIM.println(SerialMON.readStringUntil('\n'));
  //   SerialSIM.flush();
  // }
  // if (SerialSIM.available()) {
  //   SerialMON.println(SerialSIM.readStringUntil('\n'));
  //   SerialMON.flush();
  // }
}