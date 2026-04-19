#include <Arduino.h>
#include <STM32LowPower.h>
#include <EEPROM.h>
#include "SIM7000G.h"

// ─── Serial ports ────────────────────────────────────────────────
// HardwareSerial Serial2(PA3, PA2);
HardwareSerial SerialMON(PA3, PA2);
HardwareSerial SerialSIM(PB7, PB6);

SIM7000G simcom(SerialSIM, &SerialMON);

// ─── Pins ────────────────────────────────────────────────────────
#define GSM_PWR_PIN  PB8   // modem power key (active-high pulse)
#define V33_PWR_PIN  PA11  // 3.3V rail (HIGH=on)
#define V42_PWR_PIN  PA12  // 4.2V rail for GSM (HIGH=on)
#define WKUP_PIN     PC13  // button / wakeup (active-LOW, pull-up)

// ─── Device config ───────────────────────────────────────────────
#define DEVICE_WELL     0
#define DEVICE_CANAL    1
#define DEVICE_TYPE    DEVICE_CANAL   // <-- change to DEVICE_CANAL for canal

// APN for your SIM card (e.g. "internet", "uzmobile", "ucell")
#define APN            "internet"

// How many times per day to send data (2–24).
// Examples: 2→every 12h, 4→every 6h, 24→every hour
#define SENDS_PER_DAY  1440
#define SLEEP_MS       (86400000UL / SENDS_PER_DAY)

#define LONG_PRESS_MS  3000    // hold > 3s = long press (save GPS)
#define GPS_TRACK_SECS 120     // GPS mode auto-exit after 120s
#define GPS_SEND_MS    5000    // GPS mode: send interval

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
  SerialMON.print("Sleeping "); SerialMON.print(ms / 1000); SerialMON.println("s");
  SerialMON.flush();
  digitalWrite(V33_PWR_PIN, LOW);
  digitalWrite(V42_PWR_PIN, LOW);

  // Clear stale wakeup flag, then enable PC13 (WKUP2) wakeup.
  // STM32L151 standby wakes on RISING edge only:
  //   INPUT_PULLUP → normally HIGH, goes LOW on press, HIGH on release.
  //   So MCU wakes when button is RELEASED (LOW→HIGH transition).
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
  SerialMON.print("GPS saved: ");
  SerialMON.print(lat, 6); SerialMON.print(", "); SerialMON.println(lon, 6);
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
  // SerialUS.begin(9600);
  digitalWrite(V33_PWR_PIN, HIGH);
  delay(1500);

  uint32_t sum = 0;
  uint8_t  cnt = 0;
  uint8_t  buf[3];
  uint32_t t = millis();

  while (millis() - t < 5000) {
    // if (SerialUS.available()) {
    //   uint8_t head = SerialUS.read();
    //   if (head != 0xFF) continue;           // wait for frame start
    //   if (SerialUS.readBytes(buf, 3) == 3) {
    //     uint16_t d = ((uint16_t)buf[0] << 8) | buf[1];
    //     uint8_t  cs = (0xFF + buf[0] + buf[1]) & 0xFF;
    //     if (cs == buf[2] && d > 0) { sum += d; cnt++; }
    //   }
    // }
  }

  digitalWrite(V33_PWR_PIN, LOW);
  return (cnt > 0) ? (uint16_t)(sum / cnt) : 0;
}

// Returns temperature in 0.1°C steps.
// TODO: replace with DS18B20 OneWire read on your data pin.
static float readTemperature() {
  return 0.0f;
}

// Returns TDS in ppm.
// TODO: connect TDS probe to an ADC pin and calibrate.
static uint16_t readTDS() {
  // uint16_t raw = analogRead(PA0); // 12-bit
  // float v = raw * 3.3f / 4095.0f;
  // return (uint16_t)(v / 2.0f * 1000.0f); // rough conversion
  return 0;
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
// Mode: short press — GPS tracking (send live coords every 5s)
// ─────────────────────────────────────────────────────────────────

static void gpsTrackingMode() {
  SerialMON.println("=== GPS TRACKING ===");
  if (!gsmInit()) { gsmPowerOff(); return; }

  simcom.enableGPS();

  uint8_t  batt = simcom.getBattPercent();
  uint8_t  rssi = simcom.getRSSI();
  uint32_t ts   = simcom.getUnixTime();

  uint32_t mode_end = millis() + (uint32_t)GPS_TRACK_SECS * 1000UL;
  uint32_t last_tx  = 0;
  bool     gps_ok   = false;

  SerialMON.println("Waiting for GPS fix...");

  while (millis() < mode_end) {
    float lat = 0.0f, lon = 0.0f;
    if (simcom.getGPS(&lat, &lon)) {
      if (!gps_ok) { SerialMON.println("GPS fixed"); gps_ok = true; }
      if (millis() - last_tx >= GPS_SEND_MS) {
        uint16_t level = readWaterLevel();
        sendData(lat, lon, batt, rssi, 0, 0.0f, level, ts);
        ts      += GPS_SEND_MS / 1000;
        last_tx  = millis();
      }
    }
    delay(500);
  }

  simcom.disableGPS();
  gsmPowerOff();
}

// ─────────────────────────────────────────────────────────────────
// Mode: long press — get GPS fix and save to EEPROM
// ─────────────────────────────────────────────────────────────────

static void gpsSaveMode() {
  SerialMON.println("=== GPS SAVE ===");
  if (!gsmInit()) { gsmPowerOff(); return; }

  simcom.enableGPS();
  SerialMON.println("Waiting for GPS fix (90s max)...");

  float lat = 0.0f, lon = 0.0f;
  uint32_t t = millis();
  while (millis() - t < 90000UL) {
    if (simcom.getGPS(&lat, &lon)) {
      saveGPS(lat, lon);
      break;
    }
    delay(2000);
  }

  if (lat == 0.0f && lon == 0.0f)
    SerialMON.println("GPS fix failed");

  simcom.disableGPS();
  gsmPowerOff();
}

// ─────────────────────────────────────────────────────────────────
// Mode: normal scheduled wake — read sensors and send to server
// ─────────────────────────────────────────────────────────────────

static void normalDataMode() {
  SerialMON.println("=== NORMAL DATA ===");

  if (!loadGPS(&s_lat, &s_lon)) {
    SerialMON.println("No GPS in EEPROM, use default");
    s_lat = 41.299496f;  // default: Tashkent area
    s_lon = 69.240073f;
  }

  uint16_t level = readWaterLevel();
  float    temp  = readTemperature();
  uint16_t tds   = (DEVICE_TYPE == DEVICE_WELL) ? readTDS() : 0;

  SerialMON.print("Level mm: "); SerialMON.println(level);
  SerialMON.print("Temp:     "); SerialMON.println(temp);
  if (DEVICE_TYPE == DEVICE_WELL) {
    SerialMON.print("TDS:      "); SerialMON.println(tds);
  }

  if (!gsmInit()) { gsmPowerOff(); return; }

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

  digitalWrite(V33_PWR_PIN, LOW);
  digitalWrite(V42_PWR_PIN, LOW);
  digitalWrite(GSM_PWR_PIN, LOW);

  SerialMON.begin(115200);
  SerialSIM.begin(115200);
  LowPower.begin();

  SerialMON.println("\n--- BOOT ---");
  bool from_button = false;
  bool from_standby = true;
  if (from_standby)
    SerialMON.println(from_button ? "Wake: BUTTON" : "Wake: RTC");
  else
    SerialMON.println("Wake: COLD BOOT");

  // ── Diagnostic passthrough: cold boot + hold PC13 5s ────────────
  if (!from_standby && digitalRead(WKUP_PIN) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(WKUP_PIN) == LOW && millis() - t0 < 5000);
    if (millis() - t0 >= 5000) {
      SerialMON.println("=== AT PASSTHROUGH ===");
      gsmPowerOn();
      while (true) {
        if (SerialMON.available()) {
          String cmd = SerialMON.readStringUntil('\n');
          cmd.trim();
          SerialSIM.println(cmd);
          SerialSIM.flush();
        }
        while (SerialSIM.available()) SerialMON.write(SerialSIM.read());
      }
    }
  }

  if (from_standby && from_button) {
    SerialMON.println("Press again within 3s to save GPS to EEPROM...");
    bool second = false;
    uint32_t t0 = millis();
    while (millis() - t0 < LONG_PRESS_MS) {
      if (digitalRead(WKUP_PIN) == LOW) {
        delay(30);
        if (digitalRead(WKUP_PIN) == LOW) {
          second = true;
          while (digitalRead(WKUP_PIN) == LOW);
          break;
        }
      }
    }
    SerialMON.println(second ? "-> GPS SAVE" : "-> GPS TRACKING");
    if (second) gpsSaveMode();
    else        gpsTrackingMode();
  } else {
    normalDataMode();
  }

  goSleep(SLEEP_MS);
}

void loop() {}
