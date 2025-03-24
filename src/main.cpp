#include "sim800.h"
#include "STM32LowPower.h"
#include <EEPROM.h>

#define TDS_PIN PB1
#define NTC_PIN PB0
#define BMP_PIN PB14
#define V33_PWR_PIN PB4
#define V42_PWR_PIN PB5
#define VREF 3.3
#define ADC_MAX 1024
#define READ_TIME 40
#define SCOUNT 20
#define TIME_SLEEP 3600  // seconds
#define GSM_PWR_PIN PB3
#define NTC_R1 10000  // 10kOhm resistor from 3.3V

extern HardwareSerial Serial3;

SIM800 sim;

float tdsValue = 0.0;
float salinity = 0.0;
float temperature = 16.0;
float level = 0.0;
float pressure_kPa = 0.0;
String device_id = "869951000000000";
String coordinate = "40.141660,65.365109";
String url_server = "http://goldenwell.uz/api/api.php?", url;
uint32_t per_second = 0, per_minute = 0, per_hour = 0, cur_time = 0, first_15sec = 0;
bool http_request = false, first_request = false, read_sensors = true;
int httpcode = 0;
uint16_t bat_voltage = 3700;

String url_make(String id, String location, float level, float salinity, float pressure, float temperature, float ph = 5.74, int power = 100, int rssi = 3) {
  return (
    url_server + "id=" + id + "&location=" + location + "&level=" + String(level) + "&salinity=" + String(salinity) + "&pressure=" + String(pressure) + "&temperature=" + String(temperature) + "&ph=" + String(ph) + "&power=" + String(power) + "&signal=" + String(rssi));
}
void (*resetFunc)(void) = 0;
void readSensorsData();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void time_counter();
int getMedianNum(int bArray[], int iFilterLen);

void setup() {
  pinMode(V33_PWR_PIN, OUTPUT);
  pinMode(V42_PWR_PIN, OUTPUT);
  digitalWrite(V33_PWR_PIN, 0);
  digitalWrite(V42_PWR_PIN, 0);
  delay(5000);
  sim.begin(9600);
  device_id = sim.getIMEI();
  if (sim.checkPin() != errors(READY)) {
    while (1)
      ;
  }
  delay(3000);
  sim.wakeup();
  first_15sec = millis();
}

void loop() {
  if (read_sensors) {
    readSensorsData();
  }
  time_counter();
  if (http_request) {
    sim.openBearer();
    delay(3000);
    int percent = sim.getBattPercent();
    int rssi = sim.getRSSI();
    String addr = sim.getIPAddress();
    httpcode = 0;
    uint8_t req_count = 0;
    while (httpcode != 200) {
      if (!sim.getInternetStatus()) {
        sim.closeBearer();
        delay(3000);
        sim.openBearer();
        delay(3000);
      }
      url = url_make(device_id, coordinate, level, salinity, pressure_kPa, temperature, salinity * 5.2, percent, rssi);
      if (url.length() >= 100) {
        httpcode = sim.httpGet(url.c_str());
        req_count++;
      }
      if (req_count >= 3) break;
      delay(5000);
    }
    sim.closeBearer();
    digitalWrite(V33_PWR_PIN, 1);
    digitalWrite(V42_PWR_PIN, 1);
    read_sensors = false;
    http_request = false;
    LowPower.deepSleep(TIME_SLEEP * 1000 * 12);
    NVIC_SystemReset();
  }
}

void time_counter() {
  if (millis() - first_15sec >= 30000 && !first_request) {
    http_request = true;
    first_request = true;
  }
  if (millis() - cur_time >= 1000) {
    cur_time = millis();
    per_second++;
    if (per_second >= 60) {
      per_second = 0;
      per_minute++;
      if (per_minute >= 60) {
        per_minute = 0;
        per_hour++;
        resetFunc();
        http_request = true;
        if (per_hour >= 60) per_hour = 0;
      }
    }
  }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

uint32_t readAnalogTime = 0;
uint16_t NTC_buff[SCOUNT], BMP_buff[SCOUNT];
uint8_t read_counter = 0;

float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

int TDSBuffer[SCOUNT];


void readSensorsData() {
  if (millis() - readAnalogTime >= READ_TIME) {
    readAnalogTime = millis();
    BMP_buff[read_counter] = analogRead(BMP_PIN);
    TDSBuffer[read_counter] = analogRead(TDS_PIN);
    NTC_buff[read_counter++] = analogRead(NTC_PIN);

    if (read_counter >= SCOUNT) {
      uint32_t NTC_avg = 0;
      uint32_t BMP_avg = 0;
      uint32_t TDS_avg = 0;
      for (uint8_t i = 0; i < SCOUNT; i++) {
        NTC_avg += NTC_buff[i];
        BMP_avg += BMP_buff[i];
        TDS_avg += TDSBuffer[i];
      }
      NTC_avg = NTC_avg / SCOUNT;
      float NTC_voltage = VREF * NTC_avg / ADC_MAX;
      float NTC_R2 = 2 * NTC_R1 * NTC_voltage / VREF;
      float logR2 = log(NTC_R2);
      float T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
      temperature = T - 273.15;

      BMP_avg = BMP_avg / SCOUNT;
      float BMP_voltage = BMP_avg * (float)VREF / ADC_MAX;
      level = mapfloat(BMP_voltage, 0.60, 3.00, 0.0, 500.0);
      if (level < 0.0) level = 0.0;
      if (level > 500.0) level = 500.0;
      // level = level / 100;
      TDS_avg = TDS_avg / SCOUNT;
      float TDS_voltage = TDS_avg * VREF / ADC_MAX;
      Serial3.print("TDS Voltage: ");
      Serial3.println(TDS_voltage);
      // float averageVoltage = (float)getMedianNum(TDSBuffer, SCOUNT) * (float)VREF / ADC_MAX;
      // float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      // float compensationVoltage = averageVoltage / compensationCoefficient;
      // tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      // salinity += mapfloat((tdsValue / 50), 0.7, 1.2, 8.0, 0.0);
      salinity += mapfloat(TDS_voltage, 0.075, 0.12, 0.0, 8.0);
      salinity /= 2;
      Serial3.print("Salinity: ");
      Serial3.println(salinity);
      if (salinity > 7.0) salinity = 7.0;
      if (salinity < 0.0) salinity = 0.0;
      if (level <= 10.0) {
        salinity = 0.0;
        level = 0.0;
      }
      pressure_kPa = (level * 98.0665) / 1000.0;
      read_counter = 0;
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
