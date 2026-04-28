#include <Arduino.h>
#include <STM32LowPower.h>
#include <EEPROM.h>
#include "SIM7000G.h"

// ─── Serial ports ────────────────────────────────────────────────
// HardwareSerial Serial2(PA3, PA2);
HardwareSerial SerialMON(PA3, PA2);
HardwareSerial SerialSIM(PB7, PB6);
// HardwareSerial SerialUS(PA3, PA2);

SIM7000G simcom(SerialSIM, &SerialMON);

// ─── Pins ────────────────────────────────────────────────────────
#define GSM_PWR_PIN  PB8   // modem power key (active-high pulse)
#define V33_PWR_PIN  PA11  // 3.3V rail (HIGH=on)
#define V42_PWR_PIN  PA12  // 4.2V rail for GSM (HIGH=on)
#define WKUP_PIN     PC13  // Wake up pin for clear GPS data in EEPORM
#define TDS_PIN      PB1   // TDS sensor pin
#define NTC_PIN      PB0   // NTC sensor pin
#define LVL_ADC_PIN  PB14  // Water level sensor 4-20mA read pin
#define MAH_ADC_PIN  PB15  // 4-20mA read pin

// ─── Device config ───────────────────────────────────────────────
#define DEVICE_WELL     0
#define DEVICE_CANAL    1
int     DEVICE_TYPE  =  DEVICE_WELL;   // <-- change to DEVICE_CANAL for canal

// APN for your SIM card (e.g. "internet", "uzmobile", "ucell")
#define APN            "internet"

// How many times per day to send data (2–24).
// Examples: 2→every 12h, 4→every 6h, 24→every hour
#define SENDS_PER_DAY  24
#define SLEEP_MS       (86400000UL / SENDS_PER_DAY)

#define GPS_FIX_TIMEOUT_MS 120000UL   // first-boot GPS acquisition budget

#define URL_SERVER     "http://aka.org.uz:7000/api/variable?"
#define URL_SERVER2     "http://hh-water.org:7000/api/variable?"

// ─── EEPROM layout ───────────────────────────────────────────────
#define EEPROM_LAT     0   // float – 4 bytes
#define EEPROM_LON     4   // float – 4 bytes
#define EEPROM_MAGIC   8   // uint8 – validity flag
#define EEPROM_MAGIC_V 0xAB

// ─── Runtime vars ────────────────────────────────────────────────
static float  s_lat  = 0.0f;
static float  s_lon  = 0.0f;
static String s_imei = "";

// ====== MCU sozlamalari ======
#define ADC_PIN         34          // ADC kirish pin
#define ADC_MAX_VALUE   4095.0      // ESP32 = 4095, Arduino UNO = 1023
#define ADC_VREF        3.3         // ADC ma'lumot kuchlanishi (V)

// ====== Kalibrlash konstantalari ======
// Ushbu qiymatlarni standart eritma bilan (masalan 1413 µS/cm KCl) tekshirib moslang
float K_CELL     = 1.00;            // Zond konstantasi (cm⁻¹)
float TDS_FACTOR = 0.64;            // EC→TDS: NaCl=0.5, aralash=0.64, KCl=0.7
float CALIB_GAIN = 1.00;            // Umumiy kalibrlash koeffitsienti


// ─────────────────────────────────────────────────────────────────
// Power helpers
// ─────────────────────────────────────────────────────────────────

static void gsmPowerOn() {
  digitalWrite(V42_PWR_PIN, HIGH);    // enable GSM power rail
  delay(500);
  digitalWrite(GSM_PWR_PIN, HIGH);   // press power key
  delay(2000);
  digitalWrite(GSM_PWR_PIN, LOW);    // release
  delay(5000);                        // wait for modem boot
}

static void gsmPowerOff() {
  simcom.closeBearer();
  digitalWrite(GSM_PWR_PIN, HIGH);   // press power key to turn off
  delay(1500);
  digitalWrite(GSM_PWR_PIN, LOW);
  delay(1000);
  digitalWrite(V42_PWR_PIN, LOW);   // cut GSM power rail
}

static void goSleep(uint32_t ms) {
  // SerialMON.print("Sleeping "); SerialMON.print(ms / 1000); SerialMON.println("s");
  // SerialMON.flush();
  digitalWrite(V33_PWR_PIN, LOW);
  digitalWrite(V42_PWR_PIN, LOW);

  // RTC-only wake from standby. LowPower.shutdown() sets the RTC wakeup timer
  // and enters standby; MCU resets on wake.
  LowPower.shutdown(ms);
  NVIC_SystemReset();
}

// ─────────────────────────────────────────────────────────────────
// EEPROM GPS persistence
// ─────────────────────────────────────────────────────────────────

static void saveGPS(float lat, float lon) {
  EEPROM.put(EEPROM_LAT, lat);
  EEPROM.put(EEPROM_LON, lon);
  EEPROM.put(EEPROM_MAGIC, (uint8_t)EEPROM_MAGIC_V);
  // SerialMON.print("GPS saved: ");
  // SerialMON.print(lat, 6); SerialMON.print(", "); SerialMON.println(lon, 6);
}

static bool loadGPS(float* lat, float* lon) {
  uint8_t magic = 0;
  EEPROM.get(EEPROM_MAGIC, magic);
  if (magic != EEPROM_MAGIC_V) return false;
  EEPROM.get(EEPROM_LAT, *lat);
  EEPROM.get(EEPROM_LON, *lon);
  return true;
}

// ─────────────────────────────────────────────────────────────────
// Sensors
// ─────────────────────────────────────────────────────────────────

// Returns water level in mm (ultrasonic UART sensor)
static uint16_t readWaterLevel() {
  SerialMON.end();
  delay(100);
  SerialMON.begin(9600);
  digitalWrite(V33_PWR_PIN, HIGH);
  delay(1500);

  uint32_t sum = 0;
  uint8_t  cnt = 0;
  uint8_t  buf[3];
  uint32_t t = millis();

  while (millis() - t < 5000) {
    if (SerialMON.available()) {
      uint8_t head = SerialMON.read();
      if (head != 0xFF) continue;           // wait for frame start
      if (SerialMON.readBytes(buf, 3) == 3) {
        uint16_t d = ((uint16_t)buf[0] << 8) | buf[1];
        uint8_t  cs = (0xFF + buf[0] + buf[1]) & 0xFF;
        if (cs == buf[2] && d > 0) { sum += d; cnt++; }
      }
      DEVICE_TYPE = DEVICE_CANAL;
    }
  }
  digitalWrite(V33_PWR_PIN, LOW);
  SerialMON.end();
  delay(100);
  SerialMON.begin(115200);
  SerialMON.printf("Water level readings: %d, average: %d mm\n", cnt, uint16_t(sum / cnt));
  delay(5000);
  return (cnt > 0) ? (uint16_t)(sum / cnt) : 0;
}

// ================================================================
//  Median filter funksiyasi (yordamchi)
// ================================================================
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++) {
        bTab[i] = bArray[i];
    }
    int i, j, bTemp;
    // Bubble sort
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    // Median qiymatni qaytarish
    if ((iFilterLen & 1) > 0) {
        bTemp = bTab[(iFilterLen - 1) / 2];
    } else {
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    }
    return bTemp;
}


uint16_t readWaterLevelFromADC(uint8_t adc_pin) {
  // read 4-20mA sensor
  uint32_t adc_avg = 0;
  uint8_t simple_count = 30;
  int analogBuffer[simple_count];
  for (uint8_t x = 0; x < simple_count; x++) {
    analogBuffer[x] = analogRead(adc_pin);
    delay(40);
  }
  adc_avg = getMedianNum(analogBuffer, simple_count);
  uint32_t voltage = uint32_t(float(adc_avg / 4096.0) * 3300);
  if (voltage < 600) voltage = 600;   // 4mA dan pastga o'qishlarni himoya qilish
  if (voltage > 3000) voltage = 3000; // 20mA dan yuqori o'qishlarni himoya qilish
  return map(voltage, 600, 3000, 0, 5000);
}


float readTDS(uint8_t adc_pin, float temperature_C = 25.0) {
    const int   SAMPLES        = 30;
    const int   SAMPLE_DELAY   = 40;
    
    // ===== KALIBROVKA =====
    // Empirik o'lchov (aniq rezistor qiymatlari bilan):
    //   985 Ω  → 421 ppm (kutilgan 507.6) → koef 1.206
    //   9850 Ω → 43 ppm  (kutilgan 50.8)  → koef 1.181
    //   O'rtacha: 1.19
    const float CALIB_GAIN = 1.19;
    
    // --- 1. Median bilan ADC o'qish ---
    int analogBuffer[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
        analogBuffer[i] = analogRead(adc_pin);
        delay(SAMPLE_DELAY);
    }
    int median_raw = getMedianNum(analogBuffer, SAMPLES);
    
    // --- 2. ADC → Voltage ---
    float averageVoltage = ((float)median_raw / ADC_MAX_VALUE) * ADC_VREF;
    SerialMON.print("ADC median: "); SerialMON.println(median_raw);
    SerialMON.print("Voltage:    "); SerialMON.print(averageVoltage, 4); SerialMON.println(" V");
    
    // --- 3. Temperatura kompensatsiyasi ---
    float compensationCoefficient = 1.0 + 0.02 * (temperature_C - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    SerialMON.print("Comp. V:    "); SerialMON.print(compensationVoltage, 4); SerialMON.println(" V");
    
    // --- 4. Kub polinom + kalibrovka ---
    float V = compensationVoltage;
    float tdsValue = (133.42 * V * V * V 
                    - 255.86 * V * V 
                    + 857.39 * V) * 0.5 * CALIB_GAIN;
    
    SerialMON.print("TDS:        "); SerialMON.print(tdsValue, 1); SerialMON.println(" ppm");
    
    return tdsValue;
}

// ================================================================
//  NTC HARORAT FUNKSIYASI (median filter bilan)
// ================================================================
float readNTC_celsius(uint8_t adc_pin) {
    // --- Sxema va NTC parametrlari ---
    const float R_FIXED    = 10000.0;  // Yuqoridagi rezistor — 10 kΩ
    const float R_NOMINAL  = 10000.0;  // NTC qarshiligi 25°C da — 10 kΩ
    const float T_NOMINAL  = 298.15;   // 25°C Kelvinda (25 + 273.15)
    const float BETA       = 3950.0;   // Beta koeffitsient (NTC datasheet dan!)
    const int   SAMPLES    = 31;       // toq son — median uchun afzal
    const int   SAMPLE_DELAY_US = 200; // mikrosekund, namunalar orasida
    
    digitalWrite(V33_PWR_PIN, HIGH);
    delay(100);  // NTC va ADC zanjirini barqarorlashtirish uchun
    
    // --- 1-qadam: SAMPLES ta namuna olish ---
    int analogBuffer[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
        analogBuffer[i] = analogRead(adc_pin);
        delayMicroseconds(SAMPLE_DELAY_US);
    }
    
    // --- 2-qadam: Median filter bilan o'rta qiymatni olish ---
    int median_raw = getMedianNum(analogBuffer, SAMPLES);
    
    // --- 3-qadam: ADC → Volt ---
    float v_adc = ((float)median_raw / ADC_MAX_VALUE) * ADC_VREF;
    
    // Himoya: NTC uzilgan (V ≈ VREF) yoki qisqa tutashuv (V ≈ 0V)
    if (v_adc < 0.01 || v_adc > ADC_VREF - 0.01) {
        SerialMON.println("NTC XATO: uzilgan yoki qisqa tutashuv");
        // digitalWrite(V33_PWR_PIN, LOW);
        return 0.0; // yoki -127.0 kabi maxsus qiymat qaytarish mumkin
    }
    
    // --- 4-qadam: NTC qarshiligi (voltage divider teskarisi) ---
    // Sxema: VREF → R_FIXED → ADC_PIN → NTC → GND
    // V_adc = V_supply × R_ntc / (R_fixed + R_ntc)
    // ⇒ R_ntc = R_fixed × V_adc / (V_supply − V_adc)
    float r_ntc = R_FIXED * v_adc / (ADC_VREF - v_adc);
    
    // --- 5-qadam: Beta formulasi (Steinhart-Hart soddalashtirilgan) ---
    // 1/T = 1/T₀ + (1/β) × ln(R/R₀),   T Kelvinda
    float t_kelvin = 1.0 / (1.0/T_NOMINAL + log(r_ntc/R_NOMINAL) / BETA);
    
    // --- 6-qadam: Kelvin → Celsius ---
    float t_celsius = t_kelvin - 273.15;
    
    SerialMON.print("NTC ADC:    "); SerialMON.println(median_raw);
    SerialMON.print("NTC R:      "); SerialMON.print(r_ntc, 0); SerialMON.println(" Ohm");
    SerialMON.print("Temp:       "); SerialMON.print(t_celsius, 2); SerialMON.println(" °C");
    
    digitalWrite(V33_PWR_PIN, LOW);
    return t_celsius;
}

// ─────────────────────────────────────────────────────────────────
// GSM init (call once per session)
// ─────────────────────────────────────────────────────────────────

static bool gsmInit() {
  gsmPowerOn();
  uint8_t res = simcom.begin(9600);
  if (res == 10) {  // errors::timeout — modem not responding
    SerialMON.println("GSM: no modem, skip");
    return false;
  }
  simcom.setAPN(APN);
  simcom.openBearer();
  s_imei = simcom.getIMEI();
  String ip = simcom.getIPAddress();
  SerialMON.println("IP: " + ip);
  return simcom.getInternetStatus();
}

// ─────────────────────────────────────────────────────────────────
// HTTP send
// ─────────────────────────────────────────────────────────────────

static void sendData(float lat, float lon,
                     uint8_t batt, uint8_t rssi,
                     uint16_t tds, float temp,
                     uint16_t level, uint32_t ts) {
  char url[300];
  char latStr[15], lonStr[15], tempStr[10], levelStr[10], tdsStr[10];

  // float -> string
  dtostrf(lat,  0, 6, latStr);   // 6 ta kasr
  dtostrf(lon,  0, 6, lonStr);
  dtostrf(temp, 0, 2, tempStr);  // 1 ta kasr
  dtostrf(float(level/1000.0), 0, 3, levelStr); // metrda jo'natadi.
  dtostrf(float(tds/1000.0), 0, 3, tdsStr); // gramm/Litr birlikda jo'natadi

  if (DEVICE_TYPE == DEVICE_WELL) {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&tds=%s&temp_water=%s&level_water=%s&datetime=%lu&type=well",
      URL_SERVER, s_imei.c_str(),
      latStr, lonStr, batt, rssi, tdsStr, tempStr, levelStr, (unsigned long)ts);
  } else {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&level_water=%s&datetime=%lu&type=canal",
      URL_SERVER, s_imei.c_str(),
      latStr, lonStr, batt, rssi, levelStr, (unsigned long)ts);
  }
  simcom.httpGet(url);
  if (DEVICE_TYPE == DEVICE_WELL) {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&tds=%s&temp_water=%s&level_water=%s&datetime=%lu&type=well",
      URL_SERVER2, s_imei.c_str(),
      latStr, lonStr, batt, rssi, tdsStr, tempStr, levelStr, (unsigned long)ts);
  } else {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&level_water=%s&datetime=%lu&type=canal",
      URL_SERVER2, s_imei.c_str(),
      latStr, lonStr, batt, rssi, levelStr, (unsigned long)ts);
  }
  simcom.httpGet(url);
}

// ─────────────────────────────────────────────────────────────────
// Mode: scheduled RTC wake — read sensors and send to server.
// On first run (no GPS in EEPROM) acquire a fix via SIM7000G and
// persist it; subsequent runs reuse the stored coordinates.
// ─────────────────────────────────────────────────────────────────

static void normalDataMode() {
  SerialMON.println("=== NORMAL DATA ===");
  digitalWrite(V33_PWR_PIN, HIGH);
  delay(1000);
  uint16_t level = readWaterLevel();
  float temp  = 0.0;
  float tds = 0.0;
  if (DEVICE_TYPE == DEVICE_WELL) {
    temp  = readNTC_celsius(NTC_PIN);
    tds   =  readTDS(TDS_PIN, (temp > 0.0) ? temp : 25.0); // NTC o'qish muvaffaqiyatsiz bo'lsa, 25°C deb taxmin qilamiz
    level = readWaterLevelFromADC(LVL_ADC_PIN); 
  }
  
  digitalWrite(V33_PWR_PIN, LOW);

  if (!gsmInit()) { gsmPowerOff(); return; }

  if (loadGPS(&s_lat, &s_lon)) {
    SerialMON.print("GPS from EEPROM: ");
    SerialMON.print(s_lat, 6); SerialMON.print(", "); SerialMON.println(s_lon, 6);
  } else {
    SerialMON.println("No GPS in EEPROM, acquiring fix...");
    simcom.enableGPS();
    uint32_t t0 = millis();
    bool got = false;
    while (millis() - t0 < GPS_FIX_TIMEOUT_MS) {
      if (simcom.getGPS(&s_lat, &s_lon)) {
        got = true;
        break;
      }
      delay(2000);
    }
    simcom.disableGPS();
    if (!got) {
      SerialMON.println("GPS fix failed, skip send");
      s_lat = 40.08111;
      s_lon = 65.9059;
    }
    saveGPS(s_lat, s_lon);
  }

  uint8_t  batt = simcom.getBattPercent();
  uint8_t  rssi = simcom.getRSSI();
  uint32_t ts   = simcom.getUnixTime();

  SerialMON.print("Batt%: "); SerialMON.println(batt);
  SerialMON.print("RSSI:  "); SerialMON.println(rssi);

  sendData(s_lat, s_lon, batt, rssi, tds, temp, level, ts);

  gsmPowerOff();
}

// ─────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────

void setup() {
  pinMode(V33_PWR_PIN, OUTPUT);
  pinMode(V42_PWR_PIN, OUTPUT);
  pinMode(GSM_PWR_PIN, OUTPUT);
  pinMode(WKUP_PIN, INPUT_PULLUP);
  pinMode(TDS_PIN, INPUT_ANALOG);
  pinMode(NTC_PIN, INPUT_ANALOG);
  pinMode(LVL_ADC_PIN, INPUT_ANALOG);
  pinMode(MAH_ADC_PIN, INPUT_ANALOG);

  analogReadResolution(12);  // 0-4095

  digitalWrite(V33_PWR_PIN, HIGH);
  digitalWrite(V42_PWR_PIN, LOW);
  digitalWrite(GSM_PWR_PIN, LOW);

  SerialMON.begin(115200);
  LowPower.begin();
  SerialMON.println("\n--- BOOT ---");

  EEPROM.begin();
  if (digitalRead(WKUP_PIN) == LOW) {
    EEPROM.put(EEPROM_MAGIC, 0x00);
  }

  normalDataMode();

  goSleep(SLEEP_MS);
  delay(SLEEP_MS);
}

void loop() {
  // uint32_t level = readWaterLevel();
  // if (DEVICE_TYPE == DEVICE_WELL) {
  //   SerialMON.print("\n\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
  //   float temp  = readNTC_celsius(NTC_PIN);
  //   float tds   = readTDS(TDS_PIN);
  //   level       = readWaterLevelFromADC(LVL_ADC_PIN); 
  //   delay(5000);
  // }
}
