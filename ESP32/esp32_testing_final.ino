/*
 * ESP32 - 4 Sensor Unified Environmental Monitor
 * + HiveMQ Cloud MQTT publishing
 * + Nano forwarding (Pre-formatted Screen Text Stream)
 * + WiFiManager and OTA Updates
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>      
#include <ArduinoOTA.h>       
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "time.h"
#include <math.h>

#define MQTT_MAX_PACKET_SIZE 512

#include <PubSubClient.h>
#include <MQUnifiedsensor.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BME680.h>

//======================
// HiveMQ Cloud Details
//======================
const char* mqtt_server = "4b2031eb1d3349bb8e3010d2b7208cae.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "aqinitttr";
const char* mqtt_password = "Nitttr@2026";
const char* topic = "esp32/environment";

WiFiClientSecure espClient;
PubSubClient client(espClient);

HardwareSerial nanoSerial(1);

// ─── MQ-135 Config ───────────────────────────────────────────────────────────
#define MQ135_BOARD         "ESP32"
#define MQ135_PIN           32
#define MQ135_TYPE          "MQ-135"
#define MQ135_VOLTAGE       3.3
#define MQ135_ADC_BITS      12
#define RATIO_MQ135_CLEAN_AIR   3.6

MQUnifiedsensor MQ135(MQ135_BOARD, MQ135_VOLTAGE, MQ135_ADC_BITS, MQ135_PIN, MQ135_TYPE);

bool     mq135_calibrated   = false;
bool     mq135_cal_started  = false;
int      mq135_cal_count    = 0;
float    mq135_cal_sum      = 0.0;
uint32_t mq135_cal_timer    = 0;
uint32_t mq135_read_timer   = 0;
#define  MQ135_CAL_INTERVAL  500
#define  MQ135_READ_INTERVAL 3000

float global_co2 = 0, global_nh3 = 0, global_voc = 0, global_nox = 0, global_ch4 = 0;

// ─── PMS5003 Config ──────────────────────────────────────────────────────────
HardwareSerial pmsSerial(2);

struct PMS5003Data {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};
uint32_t pms_read_timer   = 0;
#define  PMS_READ_INTERVAL  2000
PMS5003Data pmsData = {0, 0, 0};
int global_aqi = 0;

// ─── BME680 Config ───────────────────────────────────────────────────────────
Adafruit_BME680 bme;
uint32_t bme_read_timer   = 0;
#define  BME_READ_INTERVAL  2000
float global_temp = 0, global_hum = 0, global_press = 0;

// ─── MAX4466 Config ──────────────────────────────────────────────────────────
#define MAX4466_PIN         34
uint32_t sound_read_timer = 0;
#define  SOUND_READ_INTERVAL 2000
int global_sound_peak = 0;

// ─── Nano Screen Transmission Config ─────────────────────────────────────────
// Nano now only displays pre-formatted text; ESP32 owns all formatting/switching logic.
uint32_t screen_timer = 0;
#define  SCREEN_INTERVAL 4000
int currentScreen = 0;
const int TOTAL_SCREENS = 22;   // 10 original screens + 12 notebook-sequence screens

// ─── OTA State ────────────────────────────────────────────────────────────────
volatile bool otaInProgress = false;

// ─── NTP / Time Config (IST, UTC+5:30) ───────────────────────────────────────
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;
const char* daysUpper[7]   = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
const char* monthsUpper[12] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};

// ─── AQI Trend Tracking ───────────────────────────────────────────────────────
int previousAQIforTrend = -1;

// ─── MQTT Publish Config ─────────────────────────────────────────────────────
unsigned long lastMsg = 0;
#define MQTT_PUBLISH_INTERVAL 3000
unsigned long lastHeapPrint = 0;

// ─── Reliability: Watchdog + Scheduled Restart ───────────────────────────────
// These are additive safety nets only — they don't change sensor/WiFi/MQTT behavior.
// Watchdog: auto-recovers the board if loop() ever hangs (e.g. a sensor library stall).
// Daily restart: clears any long-term heap fragmentation from weeks of continuous uptime.
#define WDT_TIMEOUT_SEC      15
#define RESTART_INTERVAL_MS  (24UL * 60UL * 60UL * 1000UL)  // 24 hours

const char* mqttStateToString(int state) {
  switch (state) {
    case -4: return "MQTT_CONNECTION_TIMEOUT";
    case -3: return "MQTT_CONNECTION_LOST";
    case -2: return "MQTT_CONNECT_FAILED";
    case -1: return "MQTT_DISCONNECTED";
    case  0: return "MQTT_CONNECTED";
    case  1: return "MQTT_CONNECT_BAD_PROTOCOL";
    case  2: return "MQTT_CONNECT_BAD_CLIENT_ID";
    case  3: return "MQTT_CONNECT_UNAVAILABLE";
    case  4: return "MQTT_CONNECT_BAD_CREDENTIALS";
    case  5: return "MQTT_CONNECT_UNAUTHORIZED";
    default: return "UNKNOWN";
  }
}

void flushPMS() {
  while (pmsSerial.available()) pmsSerial.read();
}

bool readPMS(PMS5003Data &d) {
  if (pmsSerial.available() < 32) return false;
  if (pmsSerial.read() != 0x42) { flushPMS(); return false; }
  if (pmsSerial.read() != 0x4D) { flushPMS(); return false; }

  uint8_t buf[30];
  pmsSerial.readBytes(buf, 30);

  uint16_t calcSum = 0x42 + 0x4D;
  for (int i = 0; i < 28; i++) calcSum += buf[i];
  uint16_t recvSum = (buf[28] << 8) | buf[29];
  if (calcSum != recvSum) {
    flushPMS();
    return false; 
  }

  d.pm1_0 = (buf[8]  << 8) | buf[9];
  d.pm2_5 = (buf[10] << 8) | buf[11];
  d.pm10  = (buf[12] << 8) | buf[13];

  return true;
}

int calcAQI(float pm2_5) {
  if (pm2_5 <= 12.0)  return (int)map((long)pm2_5,   0,    12,  0,   50);
  if (pm2_5 <= 35.4)  return (int)map((long)pm2_5,  12,    35, 51,  100);
  if (pm2_5 <= 55.4)  return (int)map((long)pm2_5,  35,    55, 101, 150);
  if (pm2_5 <= 150.4) return (int)map((long)pm2_5,  55,   150, 151, 200);
  return 201;
}

void reconnectMQTT() {
  if (!client.connected()) {
    Serial.print("Connecting to HiveMQ... ");
    String clientId = "ESP32-" + WiFi.macAddress();
    clientId.replace(":", "");

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed. State = ");
      Serial.println(mqttStateToString(client.state()));
    }
  }
}

void handleMQ135Calibration() {
  if (mq135_calibrated || !mq135_cal_started) return;
  if (millis() - mq135_cal_timer >= MQ135_CAL_INTERVAL) {
    mq135_cal_timer = millis();
    MQ135.update();
    mq135_cal_sum += MQ135.calibrate(RATIO_MQ135_CLEAN_AIR);
    mq135_cal_count++;
    if (mq135_cal_count >= 10) {
      float r0 = mq135_cal_sum / 10.0;
      if (isinf(r0) || r0 == 0) { mq135_cal_started = false; return; }
      MQ135.setR0(r0);
      mq135_calibrated = true;
      Serial.println("[MQ135] Calibration complete.");
    }
  }
}

void handleMQ135Read() {
  if (!mq135_calibrated) return;
  if (millis() - mq135_read_timer < MQ135_READ_INTERVAL) return;
  mq135_read_timer = millis();

  MQ135.update();
  
  float val;
  MQ135.setA(110.47); MQ135.setB(-2.862);  val = MQ135.readSensor(); global_co2 = (isnan(val) || isinf(val)) ? 0.0 : val;
  MQ135.setA(102.2);  MQ135.setB(-2.473);  val = MQ135.readSensor(); global_nh3 = (isnan(val) || isinf(val)) ? 0.0 : val;
  MQ135.setA(34.66);  MQ135.setB(-3.369);  val = MQ135.readSensor(); global_voc = (isnan(val) || isinf(val)) ? 0.0 : val;
  MQ135.setA(44.95);  MQ135.setB(-3.445);  val = MQ135.readSensor(); global_nox = (isnan(val) || isinf(val)) ? 0.0 : val;
  MQ135.setA(605.18); MQ135.setB(-3.937);  val = MQ135.readSensor(); global_ch4 = (isnan(val) || isinf(val)) ? 0.0 : val;
}

void handlePMS() {
  if (millis() - pms_read_timer < PMS_READ_INTERVAL) return;
  pms_read_timer = millis();
  if (readPMS(pmsData)) {
    global_aqi = calcAQI(pmsData.pm2_5);
  }
}

void handleBME680() {
  if (millis() - bme_read_timer < BME_READ_INTERVAL) return;
  bme_read_timer = millis();
  if (bme.performReading()) {
    global_temp = bme.temperature;
    global_hum = bme.humidity;
    global_press = bme.pressure / 100.0;
  }
}

void handleMAX4466() {
  if (millis() - sound_read_timer < SOUND_READ_INTERVAL) return;
  sound_read_timer = millis();

  unsigned int signalMax = 0;
  unsigned int signalMin = 4095;
  uint32_t start = millis();

  while (millis() - start < 50) {
    int v = analogRead(MAX4466_PIN);
    if (v < 4095) {
      if (v > signalMax) signalMax = v;
      if (v < signalMin) signalMin = v;
    }
  }

  unsigned int peakToPeak = signalMax - signalMin;
  int db = map(peakToPeak, 10, 1500, 25, 95);
  if (db < 20) db = 20;
  if (db > 95) db = 95;

  global_sound_peak = db;
}


char* trimLeadingSpaces(char* str) {
  while (*str == ' ') {
    str++;
  }
  return str;
}

// ─── Time/Date string builder for the TIME/DATE loop screen ─────────────────
void getTimeDateStrings(char* timeStr, size_t tsz, char* dateStr, size_t dsz) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 200)) {
    strncpy(timeStr, "TIME N/A", tsz - 1); timeStr[tsz - 1] = '\0';
    strncpy(dateStr, "--- ---", dsz - 1); dateStr[dsz - 1] = '\0';
    return;
  }
  int hour12 = timeinfo.tm_hour % 12;
  if (hour12 == 0) hour12 = 12;
  const char* ampm = (timeinfo.tm_hour < 12) ? "AM" : "PM";
  snprintf(timeStr, tsz, "%d:%02d %s %s", hour12, timeinfo.tm_min, ampm, daysUpper[timeinfo.tm_wday]);
  snprintf(dateStr, dsz, "%02d %s %d", timeinfo.tm_mday, monthsUpper[timeinfo.tm_mon], timeinfo.tm_year + 1900);
}

// ─── CPCB-style AQI category (India National AQI bands) ─────────────────────
const char* getAQICategoryCPCB(int aqi) {
  if (aqi <= 50)  return "GOOD";
  if (aqi <= 100) return "SATISFAC";
  if (aqi <= 200) return "MODERATE";
  if (aqi <= 300) return "POOR";
  if (aqi <= 400) return "VERY POOR";
  return "SEVERE";
}

// ─── AQI trend vs the previous time this screen was shown ───────────────────
const char* computeTrend(int currentAQI) {
  const char* trend;
  if (previousAQIforTrend < 0) {
    trend = "STABLE";
  } else {
    int diff = currentAQI - previousAQIforTrend;
    if (diff > 5) trend = "RISING";
    else if (diff < -5) trend = "FALLING";
    else trend = "STABLE";
  }
  previousAQIforTrend = currentAQI;
  return trend;
}

// ─── Feels-like temperature (Australian BOM apparent-temp formula, no wind term) ──
float computeFeelsLike(float tempC, float humPct) {
  float e = (humPct / 100.0) * 6.105 * exp((17.27 * tempC) / (237.7 + tempC));
  return tempC + 0.33 * e - 4.00;
}

// ─── Simple comfort banding from temp + humidity ─────────────────────────────
const char* computeComfort(float tempC, float humPct) {
  if (tempC >= 20 && tempC <= 28 && humPct >= 30 && humPct <= 60) return "GOOD";
  return "POOR";
}

// ─── Boot-time screen sender: prints one packet, then holds it on-screen ────
void sendBootScreen(const char* upper, const char* lower, unsigned long holdMs) {
  nanoSerial.print("<");
  nanoSerial.print(upper);
  nanoSerial.print("|");
  nanoSerial.print(lower);
  nanoSerial.println(">");
  delay(holdMs);
}

// ─── WiFiManager callback: fires only if the device has to open the config portal ──
void configModeCallback(WiFiManager *myWiFiManager) {
  sendBootScreen("WIFI PORTAL", "AQI_NITTTR", 200);
  Serial.println("[WiFiManager] Config portal opened");
}
//==================================
// Screen Formatting + Nano Transmission
// (Replaces old raw-CSV sendDataToNano(); Nano now just displays these two lines)
//==================================
void sendScreenToNano() {
  if (millis() - screen_timer < SCREEN_INTERVAL) return;
  screen_timer = millis();

  char cUpper[32] = "DATA WAIT";
  char cLower[32] = "SYSTEM LIVE";
  char fBuf1[12], fBuf2[12];

  switch (currentScreen) {
    case 0:
      strcpy(cUpper, "WELCOME TO");
      strcpy(cLower, "NITTTR BHOPAL");
      break;

    case 1:
      dtostrf(isnan(global_temp) ? 0.0 : global_temp, 4, 1, fBuf1);
      snprintf(cUpper, sizeof(cUpper), "TEMP:%sC", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "HUMID:%d%%", (int)(isnan(global_hum) ? 0.0 : global_hum));
      break;

    case 2:
      snprintf(cUpper, sizeof(cUpper), "SOUND:%ddB", global_sound_peak);
      if (global_sound_peak <= 30) {
        strcpy(cLower, "NOISE:QUIET");
      } else if (global_sound_peak <= 65) {
        strcpy(cLower, "NOISE:MODERATE");
      } else if (global_sound_peak <= 85) {
        strcpy(cLower, "NOISE:NOISY");
      } else {
        strcpy(cLower, "NOISE:LOUD!");
      }
      break;

    case 3:
      snprintf(cUpper, sizeof(cUpper), "AQI:%d", global_aqi);
      snprintf(cLower, sizeof(cLower), "PM1.0:%dug", pmsData.pm1_0);
      break;

    case 4:
      snprintf(cUpper, sizeof(cUpper), "PM2.5:%dug", pmsData.pm2_5);
      snprintf(cLower, sizeof(cLower), "PM10:%dug", pmsData.pm10);
      break;

    case 5:
      dtostrf(global_co2, 4, 0, fBuf1);
      dtostrf(global_nh3, 3, 1, fBuf2);
      snprintf(cUpper, sizeof(cUpper), "CO2:%sppm", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "NH3:%sppm", trimLeadingSpaces(fBuf2));
      break;

    case 6:
      dtostrf(global_voc, 3, 1, fBuf1);
      dtostrf(global_nox, 3, 1, fBuf2);
      snprintf(cUpper, sizeof(cUpper), "VOC:%sppm", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "NOx:%sppm", trimLeadingSpaces(fBuf2));
      break;

    case 7:
      dtostrf(global_ch4, 3, 1, fBuf1);
      snprintf(cUpper, sizeof(cUpper), "CH4:%sppm", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "BARO:%dhPa", (int)(isnan(global_press) ? 0.0 : global_press));
      break;

    case 8:
      strcpy(cUpper, "AIR QUALITY");
      if (global_aqi <= 50) {
        strcpy(cLower, "ENV: HEALTHY");
      } else if (global_aqi <= 100) {
        strcpy(cLower, "ENV:MODERATE");
      } else if (global_aqi <= 150) {
        strcpy(cLower, "STAY INDOOR");
      } else {
        strcpy(cLower, "ENV:HAZARD");
      }
      break;

    case 9:
      strcpy(cUpper, "SYS HEALTH");
      if (global_temp == 0.0 && global_press == 0.0) {
        strcpy(cLower, "ERR: BME680");
      } else if (pmsData.pm1_0 == 0 && pmsData.pm2_5 == 0 && pmsData.pm10 == 0) {
        strcpy(cLower, "ERR: PMS5003");
      } else {
        strcpy(cLower, "ALL SENSORS OK");
      }
      break;

    // ── Notebook-sequence screens (new) ──────────────────────────────────
    case 10: // A: WELCOME TO / NITTTR
      strcpy(cUpper, "WELCOME TO");
      strcpy(cLower, "NITTTR");
      break;

    case 11: { // B: TIME / DATE
      char timeStr[20], dateStr[20];
      getTimeDateStrings(timeStr, sizeof(timeStr), dateStr, sizeof(dateStr));
      strncpy(cUpper, timeStr, sizeof(cUpper) - 1); cUpper[sizeof(cUpper) - 1] = '\0';
      strncpy(cLower, dateStr, sizeof(cLower) - 1); cLower[sizeof(cLower) - 1] = '\0';
      break;
    }

    case 12: { // C: AQI category + TREND
      const char* cat = getAQICategoryCPCB(global_aqi);
      const char* trend = computeTrend(global_aqi);
      snprintf(cUpper, sizeof(cUpper), "AQI:%d %s", global_aqi, cat);
      snprintf(cLower, sizeof(cLower), "TREND:%s", trend);
      break;
    }

    case 13: // D: PM2.5 / PM10
      snprintf(cUpper, sizeof(cUpper), "PM2.5:%dug", pmsData.pm2_5);
      snprintf(cLower, sizeof(cLower), "PM10:%dug", pmsData.pm10);
      break;

    case 14: // E: Advisory (reuses the existing ENV logic from case 8, repositioned here)
      strcpy(cUpper, "AIR QUALITY");
      if (global_aqi <= 50) {
        strcpy(cLower, "ENV: HEALTHY");
      } else if (global_aqi <= 100) {
        strcpy(cLower, "ENV:MODERATE");
      } else if (global_aqi <= 150) {
        strcpy(cLower, "STAY INDOOR");
      } else {
        strcpy(cLower, "ENV:HAZARD");
      }
      break;

    case 15: { // F: FEEL / COMFRT
      float feels = computeFeelsLike(isnan(global_temp) ? 0.0 : global_temp, isnan(global_hum) ? 0.0 : global_hum);
      dtostrf(feels, 4, 1, fBuf1);
      snprintf(cUpper, sizeof(cUpper), "FEEL:%sC", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "COMFRT:%s", computeComfort(global_temp, global_hum));
      break;
    }

    case 16: // G: TEMP / HUM
      dtostrf(isnan(global_temp) ? 0.0 : global_temp, 4, 1, fBuf1);
      snprintf(cUpper, sizeof(cUpper), "TEMP:%sC", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "HUM:%d%%", (int)(isnan(global_hum) ? 0.0 : global_hum));
      break;

    case 17: // H: PRES / SOUND
      dtostrf(isnan(global_press) ? 0.0 : global_press, 5, 1, fBuf1);
      snprintf(cUpper, sizeof(cUpper), "PRES:%shPa", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "SOUND:%d.0dB", global_sound_peak);
      break;

    case 18: // I: NOISE LEVEL (standalone)
      strcpy(cUpper, "NOISE LEVEL:");
      if (global_sound_peak <= 30) {
        strcpy(cLower, "QUIET");
      } else if (global_sound_peak <= 65) {
        strcpy(cLower, "MODERATE");
      } else if (global_sound_peak <= 85) {
        strcpy(cLower, "NOISY");
      } else {
        strcpy(cLower, "LOUD!");
      }
      break;

    case 19: // J: CO2 / NH3
      dtostrf(global_co2, 4, 0, fBuf1);
      dtostrf(global_nh3, 3, 1, fBuf2);
      snprintf(cUpper, sizeof(cUpper), "CO2:%sppm", trimLeadingSpaces(fBuf1));
      snprintf(cLower, sizeof(cLower), "NH3:%sppm", trimLeadingSpaces(fBuf2));
      break;

    case 20: { // K: SMOKE (avg of PM2.5+PM10) / NOX
      int smokeVal = (int)round((pmsData.pm2_5 + pmsData.pm10) / 2.0);
      dtostrf(global_nox, 3, 2, fBuf2);
      snprintf(cUpper, sizeof(cUpper), "SMOKE:%dppm", smokeVal);
      snprintf(cLower, sizeof(cLower), "NOX:%sppm", trimLeadingSpaces(fBuf2));
      break;
    }

    case 21: { // L: PM1.0 / UPTIME
      unsigned long upSec = millis() / 1000UL;
      unsigned int uH = upSec / 3600;
      unsigned int uM = (upSec % 3600) / 60;
      unsigned int uS = upSec % 60;
      snprintf(cUpper, sizeof(cUpper), "PM1.0:%dug", pmsData.pm1_0);
      snprintf(cLower, sizeof(cLower), "UPTIME:%02u:%02u:%02u", uH, uM, uS);
      break;
    }
  }

  // Pre-formatted packet: "<upperLine|lowerLine>"
  // Nano only needs to split on '|' and draw — no numeric parsing on its side.
  nanoSerial.print("<");
  nanoSerial.print(cUpper);
  nanoSerial.print("|");
  nanoSerial.print(cLower);
  nanoSerial.println(">");

  currentScreen++;
  if (currentScreen >= TOTAL_SCREENS) {
    currentScreen = 0;
  }
}

//======================
// Publish to HiveMQ
//======================
void publishToHiveMQ() {
  if (WiFi.status() != WL_CONNECTED || !client.connected()) return;
  if (millis() - lastMsg < MQTT_PUBLISH_INTERVAL) return;
  lastMsg = millis();

  char payload[300];
  snprintf(payload, sizeof(payload),
    "{\"temp\":%.1f,\"hum\":%.1f,\"press\":%.1f,"
    "\"pm1\":%u,\"pm25\":%u,\"pm10\":%u,\"aqi\":%d,"
    "\"co2\":%.1f,\"nh3\":%.1f,\"voc\":%.1f,\"nox\":%.1f,\"ch4\":%.1f,"
    "\"sound\":%d}",
    global_temp, global_hum, global_press,
    (unsigned)pmsData.pm1_0, (unsigned)pmsData.pm2_5, (unsigned)pmsData.pm10, global_aqi,
    global_co2, global_nh3, global_voc, global_nox, global_ch4,
    global_sound_peak);

  client.publish(topic, payload);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println(" ESP32 Multi-Sensor + WiFiManager + OTA");
  Serial.println("========================================");

  // Nano serial must be up before WiFiManager runs, so boot screens can display
  // during the WiFi connect/portal sequence, not just after it finishes.
  nanoSerial.begin(9600, SERIAL_8N1, 5, 4);

  sendBootScreen("WELCOME TO", "NITTTR BHOPAL", 2000);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setAPCallback(configModeCallback);  // sends "WIFI PORTAL / AQI_NITTTR" if config portal opens

  bool res = wm.autoConnect("AQI_NITTTR");
  if(!res) {
    delay(3000);
    ESP.restart();
  }

  WiFi.setSleep(false);

  sendBootScreen("WIFI OK!", "CONNECTED", 2000);

  sendBootScreen("SYNCING NTP", "PLEASE WAIT", 300);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm bootTimeinfo;
  bool gotTime = false;
  for (int i = 0; i < 20 && !gotTime; i++) {
    gotTime = getLocalTime(&bootTimeinfo, 500);
  }
  char bootTimeStr[20], bootDateStr[20];
  if (gotTime) {
    getTimeDateStrings(bootTimeStr, sizeof(bootTimeStr), bootDateStr, sizeof(bootDateStr));
  } else {
    strcpy(bootDateStr, "SYNC FAILED");
  }
  sendBootScreen("NTP SYNCED", bootDateStr, 2000);

  sendBootScreen("SYSTEM READY", "NITTTR", 2000);

  ArduinoOTA.setHostname("ESP32-Env-Monitor");
  ArduinoOTA.setPassword("Nitttr2026");

  // The watchdog must not fire while OTA is writing flash. A flash erase/write
  // cycle can block loop() for longer than WDT_TIMEOUT_SEC, which panics the
  // watchdog and resets the board mid-transfer -> "connection forcibly closed"
  // on the uploader side. Pause supervision for the duration of the OTA update.
  ArduinoOTA.onStart([]() {
    otaInProgress = true;

    // Flash erase/write causes a brief voltage dip that can falsely trigger
    // the brownout detector and reset the board mid-upload. Disable it only
    // for the duration of the OTA write.
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // Free the RAM held by the MQTT/TLS session so OTA has headroom for its
    // own write buffer — WiFiClientSecure can hold 30-40KB while connected.
    client.disconnect();
    espClient.stop();

    esp_task_wdt_delete(NULL);   // stop watching this task
    Serial.println("[OTA] Start - watchdog paused, brownout disabled, MQTT freed");
  });
  ArduinoOTA.onEnd([]() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);  // re-enable brownout protection
    esp_task_wdt_add(NULL);      // resume normal watchdog supervision
    otaInProgress = false;
    Serial.println("[OTA] End - watchdog resumed, brownout re-enabled");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);  // re-enable brownout protection
    esp_task_wdt_add(NULL);      // make sure watchdog resumes even if OTA fails
    otaInProgress = false;
    Serial.printf("[OTA] Error[%u] - watchdog resumed, brownout re-enabled\n", error);
  });

  ArduinoOTA.begin();

  pmsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);
  
  if (!bme.begin()) {
    Serial.println("[BME680] Sensor missing!");
  } else {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

  pinMode(MAX4466_PIN, INPUT);
  MQ135.setRegressionMethod(1);
  MQ135.init();
  mq135_cal_started = true;
  mq135_cal_timer   = millis();

  espClient.setInsecure(); 
  client.setKeepAlive(60);
  client.setSocketTimeout(15);
  client.setServer(mqtt_server, mqtt_port);

  // Watchdog: if loop() ever stalls for WDT_TIMEOUT_SEC seconds, ESP32 auto-restarts
  // ESP32 core 3.x uses a config struct instead of (seconds, panic) arguments
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);
}

void loop() {
  ArduinoOTA.handle();

  // While a firmware write is in flight, do nothing else: no sensor polling,
  // no MQTT/serial traffic, no watchdog reset (the task was deregistered in
  // onStart()). This keeps loop() free to service ArduinoOTA.handle() as
  // fast as possible and avoids touching a watchdog handle that no longer exists.
  if (otaInProgress) {
    delay(1);
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
  }

  handleMQ135Calibration();
  handleMQ135Read();
  handlePMS();
  handleBME680();
  handleMAX4466();

  sendScreenToNano();
  publishToHiveMQ();

  unsigned long now = millis();
  if (now - lastHeapPrint > 10000) {
    lastHeapPrint = now;
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  }

  // Feed the watchdog every loop — proves the board is alive and not hung
  esp_task_wdt_reset();

  // Scheduled restart every 24h to clear any long-term heap fragmentation.
  // WiFiManager reconnects automatically with saved credentials; MQTT reconnects via reconnectMQTT().
  if (now >= RESTART_INTERVAL_MS) {
    Serial.println("[MAINTENANCE] 24h uptime reached, restarting for stability...");
    delay(500);
    ESP.restart();
  }

  delay(1);
}
