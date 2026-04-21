#include <Arduino.h>
#include <STM32LowPower.h>
#include <EEPROM.h>
#include "SIM7000G.h"

// ─── Serial ports ────────────────────────────────────────────────
// HardwareSerial Serial2(PA3, PA2);
// HardwareSerial SerialMON(PA10, PA9);
HardwareSerial SerialSIM(PB7, PB6);
HardwareSerial SerialUS(PA3, PA2);

SIM7000G simcom(SerialSIM);

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
float TDS_FACTOR = 0.50;            // EC→TDS: NaCl=0.5, aralash=0.64, KCl=0.7
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
  SerialUS.begin(9600);
  digitalWrite(V33_PWR_PIN, HIGH);
  delay(1500);

  uint32_t sum = 0;
  uint8_t  cnt = 0;
  uint8_t  buf[3];
  uint32_t t = millis();

  while (millis() - t < 5000) {
    if (SerialUS.available()) {
      uint8_t head = SerialUS.read();
      if (head != 0xFF) continue;           // wait for frame start
      if (SerialUS.readBytes(buf, 3) == 3) {
        uint16_t d = ((uint16_t)buf[0] << 8) | buf[1];
        uint8_t  cs = (0xFF + buf[0] + buf[1]) & 0xFF;
        if (cs == buf[2] && d > 0) { sum += d; cnt++; }
      }
      DEVICE_TYPE = DEVICE_CANAL;
    }
  }
  digitalWrite(V33_PWR_PIN, LOW);
  return (cnt > 0) ? (uint16_t)(sum / cnt) : 0;
}

uint32_t readWaterLevelFromADC(uint8_t adc_pin) {
  // read 4-20mA sensor
  uint32_t adc_avg = 0;
  uint8_t simple_count = 30;
  for (uint8_t x = 0; x < simple_count; x++) {
    adc_avg = (adc_avg + analogRead(adc_pin)) / 2;
  }
  float voltage = float(adc_avg / 4096.0) * 3.3;
  // Sensor voltage 12VDC
  // Shunt resistor value is 150 ohm
  // I = U/R -> R_min = 4mA = 0.004A and R_max = 20mA = 0.02A
  // U_min = 0.004 * 150 = 0.6V
  // U_max = 0.02 * 150 = 3.0V
  // Sensor measure values from 0 - 5000mm
  // from 600mV to 3000mV equal to [0-5000]
  return map(long(voltage * 1000), 600, 3000, 0, 5000);
}

// Returns TDS in ppm.
// TODO: connect TDS probe to an ADC pin and calibrate.
// ================================================================
//  ASOSIY FUNKSIYA: ADC dan PPM gacha
// ================================================================

float readTDS(uint8_t adc_pin, float temperature_C = 25.0) {

  // --- Sxema konstantalari (LM324 gain zanjiridan) ---
  const float G_DRIVER    = 0.056;         // |R48/R49| = 5.6k/100k  (U14.1)
  const float V_TDS_N_PK  = G_DRIVER * ADC_VREF / 2.0;  // ≈ 0.084 V
  const float R_SENSE     = 5600.0;        // R53 qarshiligi (Ω)
  const float RECT_COEFF  = 0.6366;        // 2/π — full-wave DC o'rtacha (U14.2+U14.4)
  const int   SAMPLES     = 64;            // shovqin kamaytirish uchun

  // --- 1-qadam: ADC ni o'rtacha qiymatda o'qish ---
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(adc_pin);
    delayMicroseconds(100);
  }
  float adc_avg = (float)sum / SAMPLES;

  // --- 2-qadam: ADC qiymatidan voltga aylantirish ---
  float v_adc = (adc_avg / ADC_MAX_VALUE) * ADC_VREF;

  // Shovqin pastki chegarasi (kuchlanish juda past = zond havoda yoki toza suv)
  if (v_adc < 0.005) return 0.0;

  // --- 3-qadam: Rektifikator koeffitsientini qaytarib, TDS_P amplitudasini tiklash ---
  //  V_ADC = 0.637 × V_TDS_P,peak  →  V_TDS_P,peak = V_ADC / 0.637
  float v_tds_p = v_adc / RECT_COEFF;

  // --- 4-qadam: Voltage divider formulasidan zond qarshiligini hisoblash ---
  //  V_TDS_P = V_TDS_N × R53 / (R53 + R_probe)
  //  ⇒  R_probe = R53 × (V_TDS_N / V_TDS_P − 1)
  if (v_tds_p >= V_TDS_N_PK) return 0.0;            // qisqa tutashuv himoyasi
  float r_probe = R_SENSE * (V_TDS_N_PK / v_tds_p - 1.0);

  // Mantiqsiz qiymatlar uchun himoya
  if (r_probe <= 0.0 || r_probe > 1.0e7) return 0.0;

  // --- 5-qadam: Elektro o'tkazuvchanlik (µS/cm) ---
  //  EC (S/cm) = K_cell / R_probe;   1 S/cm = 1e6 µS/cm
  float ec_uS_cm = (K_CELL / r_probe) * 1.0e6;

  // --- 6-qadam: Temperatura kompensatsiyasi (25°C ga keltirish) ---
  //  α ≈ 0.02 (2% har gradusga, aksariyat tuzlar uchun)
  float ec_25 = ec_uS_cm / (1.0 + 0.02 * (temperature_C - 25.0));

  // --- 7-qadam: EC (µS/cm) → TDS (ppm) + kalibrlash ---
  float tds_ppm = ec_25 * TDS_FACTOR * CALIB_GAIN;

  return tds_ppm;
}

float readNTC_celsius(uint8_t adc_pin) {
  // --- Sxema va NTC parametrlari ---
  const float ADC_MAX    = 4095.0;   // ESP32 12-bit; Arduino UNO uchun 1023.0
  const float R_FIXED    = 10000.0;  // Yuqoridagi rezistor — 10 kΩ
  const float R_NOMINAL  = 10000.0;  // NTC qarshiligi 25°C da — 10 kΩ
  const float T_NOMINAL  = 298.15;   // 25°C Kelvinda (25 + 273.15)
  const float BETA       = 3950.0;   // Beta koeffitsient (NTC datasheet dan!)
  const int   SAMPLES    = 32;

  // --- 1-qadam: Shovqinni kamaytirish uchun o'rtacha ADC ---
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(adc_pin);
    delayMicroseconds(100);
  }
  float adc_avg = (float)sum / SAMPLES;

  // --- 2-qadam: ADC → Volt ---
  float v_adc = (adc_avg / ADC_MAX) * ADC_VREF;

  // Himoya: NTC uzilgan (V ≈ 3.3V) yoki qisqa tutashuv (V ≈ 0V)
  if (v_adc < 0.01 || v_adc > ADC_VREF - 0.01) return NAN;

  // --- 3-qadam: NTC qarshiligi (voltage divider teskarisi) ---
  // Sxema: 3V3 → R_FIXED → ADC_PIN → NTC → GND
  // V_adc = V_supply × R_ntc / (R_fixed + R_ntc)
  // ⇒ R_ntc = R_fixed × V_adc / (V_supply − V_adc)
  float r_ntc = R_FIXED * v_adc / (ADC_VREF - v_adc);

  // --- 4-qadam: Beta formulasi ---
  // 1/T = 1/T₀ + (1/β) × ln(R/R₀),   T Kelvinda
  float t_kelvin = 1.0 / (1.0/T_NOMINAL + log(r_ntc/R_NOMINAL) / BETA);

  // --- 5-qadam: Kelvin → Celsius ---
  return t_kelvin - 273.15;
}

// ─────────────────────────────────────────────────────────────────
// GSM init (call once per session)
// ─────────────────────────────────────────────────────────────────

static bool gsmInit() {
  gsmPowerOn();
  uint8_t res = simcom.begin(9600);
  if (res == 10) {  // errors::timeout — modem not responding
    // SerialMON.println("GSM: no modem, skip");
    return false;
  }
  simcom.setAPN(APN);
  simcom.openBearer();
  s_imei = simcom.getIMEI();
  String ip = simcom.getIPAddress();
  // SerialMON.println("IP: " + ip);
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
  char latStr[15], lonStr[15], tempStr[10];

  // float -> string
  dtostrf(lat,  0, 6, latStr);   // 6 ta kasr
  dtostrf(lon,  0, 6, lonStr);
  dtostrf(temp, 0, 1, tempStr);  // 1 ta kasr

  if (DEVICE_TYPE == DEVICE_WELL) {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&tds=%d&temp_water=%s&level_water=%d&datetime=%lu&type=well",
      URL_SERVER, s_imei.c_str(),
      latStr, lonStr, batt, rssi, tds, tempStr, level, (unsigned long)ts);
  } else {
    snprintf(url, sizeof(url),
      "%simei=%s&lat=%s&long=%s&battery=%d&rssi=%d"
      "&level_water=%d&datetime=%lu&type=canal",
      URL_SERVER, s_imei.c_str(),
      latStr, lonStr, batt, rssi, level, (unsigned long)ts);
  }
  simcom.httpGet(url);
}

// ─────────────────────────────────────────────────────────────────
// Mode: scheduled RTC wake — read sensors and send to server.
// On first run (no GPS in EEPROM) acquire a fix via SIM7000G and
// persist it; subsequent runs reuse the stored coordinates.
// ─────────────────────────────────────────────────────────────────

static void normalDataMode() {
  // SerialMON.println("=== NORMAL DATA ===");

  uint16_t level = readWaterLevel();
  float    temp  = (DEVICE_TYPE == DEVICE_WELL) ? readNTC_celsius(NTC_PIN) : 0;
  uint16_t tds   = (DEVICE_TYPE == DEVICE_WELL) ? readTDS(TDS_PIN) : 0;
  level          = (DEVICE_TYPE == DEVICE_WELL) ? readWaterLevelFromADC(LVL_ADC_PIN) : 0; 
  // SerialMON.print("Level mm: "); SerialMON.println(level);
  // if (DEVICE_TYPE == DEVICE_WELL) {
    // SerialMON.print("Temp:     "); SerialMON.println(temp);
    // SerialMON.print("TDS:      "); SerialMON.println(tds);
  // }

  if (!gsmInit()) { gsmPowerOff(); return; }

  if (loadGPS(&s_lat, &s_lon)) {
    // SerialMON.print("GPS from EEPROM: ");
    // SerialMON.print(s_lat, 6); SerialMON.print(", "); SerialMON.println(s_lon, 6);
  } else {
    // SerialMON.println("No GPS in EEPROM, acquiring fix...");
    simcom.enableGPS();
    uint32_t t0 = millis();
    bool got = false;
    while (millis() - t0 < GPS_FIX_TIMEOUT_MS) {
      if (simcom.getGPS(&s_lat, &s_lon)) {
        saveGPS(s_lat, s_lon);
        got = true;
        break;
      }
      delay(2000);
    }
    simcom.disableGPS();
    if (!got) {
      // SerialMON.println("GPS fix failed, skip send");
      gsmPowerOff();
      return;
    }
  }

  uint8_t  batt = simcom.getBattPercent();
  uint8_t  rssi = simcom.getRSSI();
  uint32_t ts   = simcom.getUnixTime();

  // SerialMON.print("Batt%: "); SerialMON.println(batt);
  // SerialMON.print("RSSI:  "); SerialMON.println(rssi);

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


  digitalWrite(V33_PWR_PIN, LOW);
  digitalWrite(V42_PWR_PIN, LOW);
  digitalWrite(GSM_PWR_PIN, LOW);

  // SerialMON.begin(115200);
  SerialSIM.begin(115200);
  LowPower.begin();
  // SerialMON.println("\n--- BOOT ---");

  EEPROM.begin();
  if (digitalRead(WKUP_PIN) == LOW) {
    EEPROM.put(EEPROM_MAGIC, 0x00);
  }

  normalDataMode();

  goSleep(SLEEP_MS);
}

void loop() {}
