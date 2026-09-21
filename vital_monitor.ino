/*
  MAX30105: SDA=6, SCL=7 (hardware I2C)
  OLED:     SDA=8, SCL=9 (software I2C via U8g2)
  PMS5003:  RX=20, TX=21
*/
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <PMS.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>
#include <sys/time.h>

//OLED
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 9, /* data=*/ 8, /* reset=*/ U8X8_PIN_NONE);

//PMS5003
#define RX_PIN 20
#define TX_PIN 21
HardwareSerial pmsSerial(1);
PMS pms(pmsSerial);
PMS::DATA pmsData;
uint16_t pm1_0 = 0, pm2_5 = 0, pm10 = 0;

// MAX30105 
MAX30105 particleSensor;

#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2;
int8_t validSPO2;
int32_t heartRateBuf;
int8_t validHeartRate;

int shownBPM = 0;
int shownSpo2 = 0;
int shownBreathingRate = 0;
bool fingerPresent = false;

// Breathing rate is not sensed directly — it is estimated from BPM and SpO2
// using the resting HR:RR ~4:1 ratio, nudged by SpO2 deviation from 98%.
float computeBreathingRate(int bpm, float spo2) {
  if (bpm <= 0 || spo2 <= 0) return 0;
  float rr = (bpm / 4.0f) + ((98.0f - spo2) * 0.5f);
  if (rr < 8) rr = 8;
  if (rr > 40) rr = 40;
  return rr;
}

bool wasFingerPresent = false;
unsigned long fingerSinceMs = 0;
const unsigned long SETTLE_MS = 3000;

const uint32_t SATURATION_LIMIT = 220000;
const uint32_t MIN_AC_AMPLITUDE = 500;
const uint32_t MOTION_LIMIT = 8000;

uint32_t noFingerBaseline = 2000;
const uint32_t FINGER_MARGIN = 8000;

const byte BPM_HIST_SIZE = 6;
int bpmHistory[BPM_HIST_SIZE];
byte bpmHistCount = 0;
byte bpmHistSpot = 0;

const byte SPO2_HIST_SIZE = 6;
int spo2History[SPO2_HIST_SIZE];
byte spo2HistCount = 0;
byte spo2HistSpot = 0;

int bpmDisagreeVal = -1;
byte bpmDisagreeStreak = 0;
int spo2DisagreeVal = -1;
byte spo2DisagreeStreak = 0;
const byte RESYNC_STREAK = 3;

const float EFFECTIVE_SAMPLE_RATE = 25.0;
const float MIN_PEAK_AC = 200.0;
const float PEAK_THRESHOLD_FRACTION = 0.62f;
const float MAX_EXPECTED_BPM = 160.0f;
int lastPeakCount = 0;
float lastAcAmplitude = 0;

uint32_t lastWindowAC = 0;

void resetHistories();
int medianOf(int *arr, byte count);
void addBpmSample(int value);
void addSpo2Sample(int value);
bool windowIsClean(uint32_t *buf);
float computeBPMFromWindow(uint32_t *buf, int len, float sampleRateHz);
void handlePMS();
void updateOLED();

/* BLE LAYER  */

#define DEVICE_NAME "VitalMonitor"

#define SERVICE_UUID           "a1b2c3d0-0001-1000-8000-00805f9b34fb"
#define CHAR_VITALS_UUID       "a1b2c3d0-0002-1000-8000-00805f9b34fb" // notify: bpm/spo2/finger
#define CHAR_CONTROL_UUID      "a1b2c3d0-0003-1000-8000-00805f9b34fb" // write:  "START" / "STOP"
#define CHAR_STATUS_UUID       "a1b2c3d0-0004-1000-8000-00805f9b34fb" // notify: measurement status incl PM
#define CHAR_HISTORY_UUID      "a1b2c3d0-0005-1000-8000-00805f9b34fb" // read/write: paged history (8/page)
#define CHAR_PROFILE_UUID      "a1b2c3d0-0006-1000-8000-00805f9b34fb" // read/write: name/age/gender
#define CHAR_HISTORYSYNC_UUID  "a1b2c3d0-0007-1000-8000-00805f9b34fb" // write: SYNC:START/SYNC:CANCEL/ACK:<n>; notify: header/record/done/error

BLEServer *pServer = nullptr;
BLECharacteristic *vitalsChar      = nullptr;
BLECharacteristic *controlChar     = nullptr;
BLECharacteristic *statusChar      = nullptr;
BLECharacteristic *historyChar     = nullptr;
BLECharacteristic *historySyncChar = nullptr;
BLECharacteristic *profileChar     = nullptr;

bool deviceConnected = false;
int  historyPage = 0;
// A BLE characteristic value must stay under 512 bytes. One record is ~75 bytes of JSON
// (more now that ts is a 10-digit unix time), so 5 records/page (~400 bytes) is safe; 8 is not.
const int HISTORY_PAGE_SIZE = 5;

bool storageOk = false;

// ---- real-time clock (set by the app over BLE on every connect) ----
// The ESP32 has no battery-backed clock, so time is lost on power-off.
// The app sends "TIME:<unix epoch seconds, UTC>" each time it connects.
bool timeSynced = false;

// ---- measurement state ----
bool measuringActive = false, measurementDone = false;
unsigned long measureStartMs = 0;
const unsigned long MEASURE_DURATION_MS = 30000UL;   // 30 seconds
int measureRemainingSec = 0;

float acc_bpm = 0; int acc_bpm_cnt = 0;
float acc_spo2 = 0; int acc_spo2_cnt = 0;
float acc_pm1 = 0, acc_pm25 = 0, acc_pm10 = 0; int acc_pm_cnt = 0;

int   avgBPM = 0; float avgSpo2 = 0, avgPM1 = 0, avgPM25 = 0, avgPM10 = 0;
bool  autoSaved = false;


File syncFile;                 // stays open for the duration of a sync, read backward
long syncReadPos = 0;           // exclusive upper bound of the still-unread region [0, syncReadPos)
String syncLastLine;            // last line sent — cached so a retry resends without re-reading
bool syncActive = false;
int syncTotal = 0;
int syncSeq = 0;
bool syncWaitingAck = false;
int syncRetries = 0;
unsigned long syncSentMs = 0;
const unsigned long SYNC_ACK_TIMEOUT_MS = 3000;

void initStorage();
void initBLE();
void loadProfileFromFS();
void saveRecord(int bpm, float spo2v, float pm1, float pm25, float pm10v);
void startMeasurement();
void stopMeasurement(bool save);
void notifyVitals();
void notifyStatus();
void startHistorySync();
void cancelHistorySync(const char* reason, bool sendError);
bool readPrevHistoryLine(String &outLine);
void sendNextSyncRecord();
void resendCurrentSyncRecord();
void serviceHistorySync();

/* ---- BLE callbacks ---- */

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println("BLE client connected.");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    cancelHistorySync("disconnected", false);
    Serial.println("BLE client disconnected. Restarting advertising.");
    BLEDevice::startAdvertising();
  }
};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    v.trim();
    if (v == "START") {
      startMeasurement();
    } else if (v == "STOP") {
      stopMeasurement(false); // aborted — do not save partial data
    } else if (v.startsWith("TIME:")) {
      long long epoch = strtoll(v.c_str() + 5, NULL, 10);
      if (epoch > 1577836800LL) {            // sanity check: after 1 Jan 2020
        struct timeval tv;
        tv.tv_sec = (time_t)epoch;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        timeSynced = true;
        Serial.printf("Time synced from app: %lld\n", epoch);
      }
    }
  }
};

class ProfileCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    if (!storageOk) return;
    File f = LittleFS.open("/profile.txt", "w");
    if (!f) return;
    f.print(v);
    f.close();
    Serial.println("Profile updated: " + v);
  }
};

class HistoryCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    if (v.startsWith("PAGE:")) {
      historyPage = v.substring(5).toInt();
      if (historyPage < 0) historyPage = 0;
    }
  }
  void onRead(BLECharacteristic *c) override {
    DynamicJsonDocument doc(2048);
    doc["page"] = historyPage;
    JsonArray arr = doc.createNestedArray("records");
    if (storageOk) {
      File f = LittleFS.open("/history.csv", "r");
      if (f) {
        // load all lines (kept small deliberately — see HistorySync for the full set)
        String lines[64]; int total = 0;
        while (f.available() && total < 64) {
          String l = f.readStringUntil('\n'); l.trim();
          if (l.length() == 0) continue;
          lines[total++] = l;
        }
        f.close();
        int startIdx = total - 1 - (historyPage * HISTORY_PAGE_SIZE);
        int count = 0;
        for (int i = startIdx; i >= 0 && count < HISTORY_PAGE_SIZE; i--, count++) {
          String l = lines[i];
          int p1=l.indexOf(','),p2=l.indexOf(',',p1+1),p3=l.indexOf(',',p2+1),
              p4=l.indexOf(',',p3+1),p5=l.indexOf(',',p4+1);
          if (p1<0||p2<0||p3<0||p4<0||p5<0) continue;
          JsonObject o = arr.createNestedObject();
          o["ts"]=l.substring(0,p1).toInt(); o["bpm"]=l.substring(p1+1,p2).toInt();
          o["spo2"]=l.substring(p2+1,p3).toFloat(); o["pm1"]=l.substring(p3+1,p4).toFloat();
          o["pm25"]=l.substring(p4+1,p5).toFloat(); o["pm10"]=l.substring(p5+1).toFloat();
        }
      }
    }
    String out; serializeJson(doc, out);
    c->setValue(out.c_str());
  }
};

class HistorySyncCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    v.trim();
    if (v == "SYNC:START") {
      startHistorySync();
    } else if (v == "SYNC:CANCEL") {
      cancelHistorySync("cancelled", false);
    } else if (v.startsWith("ACK:")) {
      int ackSeq = v.substring(4).toInt();
      if (syncActive && syncWaitingAck && ackSeq == syncSeq) {
        syncWaitingAck = false;
        syncRetries = 0;
        syncSeq++;
        if (syncSeq >= syncTotal) {
          StaticJsonDocument<32> doc;
          doc["type"] = "done";
          String out; serializeJson(doc, out);
          historySyncChar->setValue(out.c_str());
          historySyncChar->notify();
          syncActive = false;
          if (syncFile) syncFile.close();
        }
      }
      // a mismatched ack (stale/duplicate) is silently ignored — the
      // pending record will simply be resent on timeout if needed
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);

  pmsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pms.passiveMode();
  delay(1000);
  Serial.println("PMS5003 ready.");

  Wire.begin(6, 7);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Check wiring.");
    while (1);
  }

  byte ledBrightness = 0x5F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 16384;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x5F);
  particleSensor.setPulseAmplitudeGreen(0);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  Serial.println("Place your finger on the sensor.");

  for (int i = 0; i < BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                          &spo2, &validSPO2, &heartRateBuf, &validHeartRate);

  initStorage();
  initBLE();
}

void loop() {
  for (int i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  for (int i = 75; i < BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
    handlePMS();
  }

  uint32_t curIR = irBuffer[BUFFER_LENGTH - 1];
  if (curIR < 10000) {
    noFingerBaseline = (noFingerBaseline * 7 + curIR) / 8;
  }
  fingerPresent = (curIR > (noFingerBaseline + FINGER_MARGIN)) && (curIR < SATURATION_LIMIT);

  uint32_t mn = irBuffer[0], mx = irBuffer[0];
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    if (irBuffer[i] < mn) mn = irBuffer[i];
    if (irBuffer[i] > mx) mx = irBuffer[i];
  }
  lastWindowAC = mx - mn;

  if (fingerPresent && !wasFingerPresent) {
    fingerSinceMs = millis();
    resetHistories();
  }
  wasFingerPresent = fingerPresent;

  if (fingerPresent) {
    bool clean = windowIsClean(irBuffer);

    if (clean) {
      maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                              &spo2, &validSPO2, &heartRateBuf, &validHeartRate);
    }

    float bpmResult = computeBPMFromWindow(irBuffer, BUFFER_LENGTH, EFFECTIVE_SAMPLE_RATE);

    bool settled = (millis() - fingerSinceMs) >= SETTLE_MS;

    if (settled) {
      if (clean && bpmResult > 30 && bpmResult < 220) {
        addBpmSample((int)bpmResult);
      }
      if (clean && validSPO2 && spo2 > 70 && spo2 <= 100) {
        addSpo2Sample((int)spo2);
      }
      shownBPM = medianOf(bpmHistory, bpmHistCount);
      shownSpo2 = medianOf(spo2History, spo2HistCount);
      shownBreathingRate = (int)round(computeBreathingRate(shownBPM, shownSpo2));
    }
  } else {
    resetHistories();
    shownBPM = 0;
    shownSpo2 = 0;
    shownBreathingRate = 0;
  }

  Serial.print("IR=");
  Serial.print(curIR);
  Serial.print(", base=");
  Serial.print(noFingerBaseline);
  Serial.print(", winAC=");
  Serial.print(lastWindowAC);
  Serial.print(", acAmp=");
  Serial.print(lastAcAmplitude, 0);
  Serial.print(", peaks=");
  Serial.print(lastPeakCount);
  Serial.print(", BPM=");
  Serial.print(shownBPM);
  Serial.print(", SpO2=");
  Serial.print(shownSpo2);
  Serial.print("%");
  if (!fingerPresent) {
    Serial.print(" No finger?");
  } else if ((millis() - fingerSinceMs) < SETTLE_MS) {
    Serial.print(" Stabilizing...");
  } else if (lastWindowAC > MOTION_LIMIT) {
    Serial.print(" MOTION - hold still");
  } else if (lastWindowAC < MIN_AC_AMPLITUDE) {
    Serial.print(" Signal too flat - press lighter/adjust position");
  }
  Serial.println();

  if (measuringActive) {
    unsigned long elapsed = millis() - measureStartMs;
    if (elapsed >= MEASURE_DURATION_MS) {
      avgBPM  = acc_bpm_cnt  > 0 ? (int)round(acc_bpm  / acc_bpm_cnt)  : 0;
      avgSpo2 = acc_spo2_cnt > 0 ? acc_spo2 / acc_spo2_cnt             : 0;
      avgPM1  = acc_pm_cnt   > 0 ? acc_pm1  / acc_pm_cnt               : 0;
      avgPM25 = acc_pm_cnt   > 0 ? acc_pm25 / acc_pm_cnt               : 0;
      avgPM10 = acc_pm_cnt   > 0 ? acc_pm10 / acc_pm_cnt               : 0;
      stopMeasurement(true); // saves record, sets measurementDone
      Serial.printf("Measurement done: BPM=%d SpO2=%.1f PM1=%.1f PM2.5=%.1f PM10=%.1f\n",
                    avgBPM, avgSpo2, avgPM1, avgPM25, avgPM10);
    } else {
      measureRemainingSec = (MEASURE_DURATION_MS - elapsed) / 1000;
      static unsigned long lastAcc = 0;
      if (millis() - lastAcc >= 1000) {
        lastAcc = millis();
        if (shownBPM > 0)  { acc_bpm  += shownBPM;  acc_bpm_cnt++; }
        if (shownSpo2 > 0) { acc_spo2 += shownSpo2; acc_spo2_cnt++; }
        acc_pm1 += pm1_0; acc_pm25 += pm2_5; acc_pm10 += pm10; acc_pm_cnt++;
      }
    }
  }

  // ---- BLE notifications, throttled to ~1/sec ----
  static unsigned long lastNotify = 0;
  if (deviceConnected && millis() - lastNotify >= 1000) {
    lastNotify = millis();
    notifyVitals();
    if (measuringActive || measurementDone) notifyStatus();
  }

  // ---- HistorySync chunked-transfer state machine ----
  serviceHistorySync();

  updateOLED();
}

float computeBPMFromWindow(uint32_t *buf, int len, float sampleRateHz) {
  static float smoothed[BUFFER_LENGTH];
  static float ac[BUFFER_LENGTH];

  const int SMOOTH_WIN = 3;
  for (int i = 0; i < len; i++) {
    long sum = 0; int cnt = 0;
    for (int k = -SMOOTH_WIN / 2; k <= SMOOTH_WIN / 2; k++) {
      int idx = i + k;
      if (idx >= 0 && idx < len) { sum += buf[idx]; cnt++; }
    }
    smoothed[i] = (float)sum / cnt;
  }

  const int BASE_WIN = 21;
  for (int i = 0; i < len; i++) {
    long sum = 0; int cnt = 0;
    for (int k = -BASE_WIN / 2; k <= BASE_WIN / 2; k++) {
      int idx = i + k;
      if (idx >= 0 && idx < len) { sum += buf[idx]; cnt++; }
    }
    float base = (float)sum / cnt;
    ac[i] = smoothed[i] - base;
  }

  float acMin = ac[0], acMax = ac[0];
  for (int i = 1; i < len; i++) {
    if (ac[i] < acMin) acMin = ac[i];
    if (ac[i] > acMax) acMax = ac[i];
  }
  float amplitude = acMax - acMin;
  lastAcAmplitude = amplitude;

  if (amplitude < MIN_PEAK_AC) {
    lastPeakCount = 0;
    return 0;
  }

  float threshold = acMin + PEAK_THRESHOLD_FRACTION * amplitude;

  int minDistSamples = (int)(sampleRateHz * 60.0 / MAX_EXPECTED_BPM);
  if (minDistSamples < 1) minDistSamples = 1;

  int peakIndices[20];
  int peakCount = 0;
  int lastPeakIdx = -1000;

  for (int i = 1; i < len - 1; i++) {
    if (ac[i] > threshold && ac[i] >= ac[i - 1] && ac[i] >= ac[i + 1]) {
      if (i - lastPeakIdx >= minDistSamples) {
        if (peakCount < 20) peakIndices[peakCount++] = i;
        lastPeakIdx = i;
      }
    }
  }

  lastPeakCount = peakCount;

  if (peakCount < 3) return 0;

  float totalDist = 0;
  for (int i = 1; i < peakCount; i++) totalDist += (peakIndices[i] - peakIndices[i - 1]);
  float avgDist = totalDist / (peakCount - 1);

  return 60.0f * sampleRateHz / avgDist;
}

bool windowIsClean(uint32_t *buf) {
  uint32_t mn = buf[0], mx = buf[0];
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    if (buf[i] < mn) mn = buf[i];
    if (buf[i] > mx) mx = buf[i];
  }
  if (mx >= SATURATION_LIMIT) return false;
  if ((mx - mn) < MIN_AC_AMPLITUDE) return false;
  if ((mx - mn) > MOTION_LIMIT) return false;
  return true;
}

void resetHistories() {
  bpmHistCount = 0; bpmHistSpot = 0;
  spo2HistCount = 0; spo2HistSpot = 0;
  bpmDisagreeVal = -1; bpmDisagreeStreak = 0;
  spo2DisagreeVal = -1; spo2DisagreeStreak = 0;
}

void addBpmSample(int value) {
  if (bpmHistCount >= 3) {
    int med = medianOf(bpmHistory, bpmHistCount);
    if (abs(value - med) > 20) {
      if (bpmDisagreeVal >= 0 && abs(value - bpmDisagreeVal) <= 10) {
        bpmDisagreeStreak++;
      } else {
        bpmDisagreeVal = value;
        bpmDisagreeStreak = 1;
      }
      if (bpmDisagreeStreak >= RESYNC_STREAK) {
        bpmHistCount = 0; bpmHistSpot = 0;
        bpmDisagreeStreak = 0;
      } else {
        return;
      }
    } else {
      bpmDisagreeStreak = 0;
    }
  }
  bpmHistory[bpmHistSpot++] = value;
  bpmHistSpot %= BPM_HIST_SIZE;
  if (bpmHistCount < BPM_HIST_SIZE) bpmHistCount++;
}

void addSpo2Sample(int value) {
  if (spo2HistCount >= 3) {
    int med = medianOf(spo2History, spo2HistCount);
    if (abs(value - med) > 4) {
      if (spo2DisagreeVal >= 0 && abs(value - spo2DisagreeVal) <= 2) {
        spo2DisagreeStreak++;
      } else {
        spo2DisagreeVal = value;
        spo2DisagreeStreak = 1;
      }
      if (spo2DisagreeStreak >= RESYNC_STREAK) {
        spo2HistCount = 0; spo2HistSpot = 0;
        spo2DisagreeStreak = 0;
      } else {
        return;
      }
    } else {
      spo2DisagreeStreak = 0;
    }
  }
  spo2History[spo2HistSpot++] = value;
  spo2HistSpot %= SPO2_HIST_SIZE;
  if (spo2HistCount < SPO2_HIST_SIZE) spo2HistCount++;
}

int medianOf(int *arr, byte count) {
  if (count == 0) return 0;
  int tmp[8];
  for (byte i = 0; i < count; i++) tmp[i] = arr[i];
  for (byte i = 1; i < count; i++) {
    int key = tmp[i];
    byte j = i;
    while (j > 0 && tmp[j - 1] > key) {
      tmp[j] = tmp[j - 1];
      j--;
    }
    tmp[j] = key;
  }
  return tmp[count / 2];
}

void updateOLED() {
  u8g2.clearBuffer();

  if (measuringActive) {
    u8g2.setCursor(0, 10);
    u8g2.print("Measuring...");
    u8g2.setCursor(0, 24);
    u8g2.print(measureRemainingSec);
    u8g2.print("s left");
    u8g2.setCursor(0, 40);
    u8g2.print("BPM:"); u8g2.print(shownBPM);
    u8g2.print(" O2:"); u8g2.print(shownSpo2);
    u8g2.setCursor(0, 54);
    u8g2.print("PM2.5:"); u8g2.print(pm2_5);

  } else if (!deviceConnected) {
    u8g2.setCursor(0, 10);
    u8g2.print("BLE: ");
    u8g2.print(DEVICE_NAME);
    u8g2.setCursor(0, 26);
    u8g2.print("Waiting for app");
    u8g2.setCursor(0, 40);
    u8g2.print("to connect...");

  } else {
    u8g2.setCursor(0, 12);
    if (!fingerPresent) {
      u8g2.print("Place finger...");
    } else if ((millis() - fingerSinceMs) < SETTLE_MS) {
      u8g2.print("Stabilizing...");
    } else {
      u8g2.print("BPM: ");
      u8g2.print(shownBPM);
    }
    u8g2.setCursor(0, 26);
    u8g2.print("SpO2: ");
    u8g2.print(shownSpo2);
    u8g2.print("%");
    u8g2.setCursor(0, 44);
    u8g2.print("BLE: Connected");
    u8g2.setCursor(0, 58);
    u8g2.print("PM1.0:");
    u8g2.print(pm1_0);
    u8g2.print(" PM10:");
    u8g2.print(pm10);
  }
  u8g2.sendBuffer();
}

// ---- PMS logic — matches your standalone snippet exactly ----
unsigned long lastPMSRequest = 0;
bool pmsWaiting = false;

void handlePMS() {
  if (!pmsWaiting && millis() - lastPMSRequest >= 5000) {
    pms.requestRead();
    lastPMSRequest = millis();
    pmsWaiting = true;
  }
  if (pmsWaiting) {
    if (pms.read(pmsData)) {
      pm1_0 = pmsData.PM_AE_UG_1_0;
      pm2_5 = pmsData.PM_AE_UG_2_5;
      pm10  = pmsData.PM_AE_UG_10_0;
      Serial.println("---- PMS5003 reading ----");
      Serial.print("PM1.0: "); Serial.println(pm1_0);
      Serial.print("PM2.5: "); Serial.println(pm2_5);
      Serial.print("PM10 : "); Serial.println(pm10);
      pmsWaiting = false;
    } else if (millis() - lastPMSRequest > 2500) {
      Serial.println("No data from PMS5003.");
      pmsWaiting = false;
    }
  }
}

/* ================= storage / BLE setup helpers ================= */

void initStorage() {
  storageOk = LittleFS.begin(true);
  if (!storageOk) Serial.println("LittleFS mount failed.");
}

void loadProfileFromFS() {
  String content = "{\"name\":\"\",\"age\":0,\"gender\":\"\"}";
  if (storageOk && LittleFS.exists("/profile.txt")) {
    File f = LittleFS.open("/profile.txt", "r");
    if (f) {
      content = f.readString();
      f.close();
    }
  }
  profileChar->setValue(content.c_str());
}

void saveRecord(int bpm, float spo2v, float pm1, float pm25, float pm10v) {
  if (!storageOk) return;
  File f = LittleFS.open("/history.csv", "a");
  if (!f) return;
  // ts = real unix time (UTC seconds) once the app has synced the clock; 0 if never synced
  unsigned long ts = timeSynced ? (unsigned long)time(nullptr) : 0UL;
  f.printf("%lu,%d,%.1f,%.1f,%.1f,%.1f\n", ts, bpm, spo2v, pm1, pm25, pm10v);
  f.close();
}

void startMeasurement() {
  if (measuringActive) return;
  measuringActive = true; measurementDone = false;
  measureStartMs = millis();
  measureRemainingSec = MEASURE_DURATION_MS / 1000;
  acc_bpm = 0; acc_bpm_cnt = 0; acc_spo2 = 0; acc_spo2_cnt = 0;
  acc_pm1 = 0; acc_pm25 = 0; acc_pm10 = 0; acc_pm_cnt = 0;
  Serial.println("Measurement started via BLE.");
}

void stopMeasurement(bool save) {
  measuringActive = false;
  if (save) {
    measurementDone = true;
    saveRecord(avgBPM, avgSpo2, avgPM1, avgPM25, avgPM10);
    autoSaved = true;
  } else {
    measurementDone = false;
    Serial.println("Measurement stopped/aborted via BLE - not saved.");
  }
}

void notifyVitals() {
  StaticJsonDocument<160> doc;
  doc["bpm"] = shownBPM;
  doc["spo2"] = shownSpo2;
  doc["breathing"] = shownBreathingRate;
  doc["finger"] = fingerPresent;
  String out; serializeJson(doc, out);
  vitalsChar->setValue(out.c_str());
  vitalsChar->notify();
}

void notifyStatus() {
  StaticJsonDocument<320> doc;
  doc["active"] = measuringActive;
  doc["remaining"] = measureRemainingSec;
  doc["done"] = measurementDone;
  doc["liveBPM"] = shownBPM;
  doc["liveSpo2"] = shownSpo2;
  doc["liveBreathing"] = shownBreathingRate;
  doc["livePM1"] = pm1_0;
  doc["livePM25"] = pm2_5;
  doc["livePM10"] = pm10;
  if (measurementDone) {
    doc["avgBPM"] = avgBPM;
    doc["avgSpo2"] = avgSpo2;
    doc["avgBreathing"] = computeBreathingRate(avgBPM, avgSpo2);
    doc["avgPM1"] = avgPM1;
    doc["avgPM25"] = avgPM25;
    doc["avgPM10"] = avgPM10;
  }
  String out; serializeJson(doc, out);
  statusChar->setValue(out.c_str());
  statusChar->notify();
}

/* ---- HistorySync: chunked, ack'd, streamed full-history transfer ----
   Reads history.csv backward one line at a time (seek + byte scan) so
   RAM use stays O(1) regardless of history size — no full-file buffer,
   suitable even for thousands of records. */

bool readPrevHistoryLine(String &outLine) {
  if (syncReadPos <= 0) return false;

  long pos = syncReadPos - 1;

  // skip a single trailing newline at the edge of the unread region
  syncFile.seek(pos);
  char ch = (char)syncFile.read();
  if (ch == '\n') pos--;

  if (pos < 0) {
    syncReadPos = 0;
    return false;
  }

  long lineEnd = pos;
  long lineStart = pos;
  while (lineStart >= 0) {
    syncFile.seek(lineStart);
    char c = (char)syncFile.read();
    if (c == '\n') break;
    lineStart--;
  }

  long contentStart = lineStart + 1;
  int len = lineEnd - contentStart + 1;
  syncReadPos = (lineStart < 0) ? 0 : lineStart;

  if (len <= 0) return false; // stray blank line — treat as no more data here

  char buf[96];
  if (len > 95) len = 95; // safety cap, a record line is never this long
  syncFile.seek(contentStart);
  syncFile.readBytes(buf, len);
  buf[len] = '\0';
  outLine = String(buf);
  outLine.trim();
  return true;
}

void buildAndSendSyncRecord(int seq, const String &l) {
  int p1=l.indexOf(','),p2=l.indexOf(',',p1+1),p3=l.indexOf(',',p2+1),
      p4=l.indexOf(',',p3+1),p5=l.indexOf(',',p4+1);
  StaticJsonDocument<192> doc;
  doc["type"] = "record";
  doc["seq"] = seq;
  if (p1>=0&&p2>=0&&p3>=0&&p4>=0&&p5>=0) {
    doc["ts"]   = l.substring(0,p1).toInt();
    doc["bpm"]  = l.substring(p1+1,p2).toInt();
    doc["spo2"] = l.substring(p2+1,p3).toFloat();
    doc["pm1"]  = l.substring(p3+1,p4).toFloat();
    doc["pm25"] = l.substring(p4+1,p5).toFloat();
    doc["pm10"] = l.substring(p5+1).toFloat();
  }
  String out; serializeJson(doc, out);
  historySyncChar->setValue(out.c_str());
  historySyncChar->notify();
  syncSentMs = millis();
  syncWaitingAck = true;
}

void sendNextSyncRecord() {
  String l;
  if (!readPrevHistoryLine(l)) {
    // fewer usable lines than the counted total — end cleanly rather than hang
    StaticJsonDocument<32> doc;
    doc["type"] = "done";
    String out; serializeJson(doc, out);
    historySyncChar->setValue(out.c_str());
    historySyncChar->notify();
    cancelHistorySync("", false);
    return;
  }
  syncLastLine = l;
  buildAndSendSyncRecord(syncSeq, l);
}

void resendCurrentSyncRecord() {
  buildAndSendSyncRecord(syncSeq, syncLastLine);
}

void startHistorySync() {
  if (syncFile) syncFile.close(); // safety, in case a prior sync was interrupted

  syncTotal = 0;
  if (storageOk) {
    File cf = LittleFS.open("/history.csv", "r");
    if (cf) {
      while (cf.available()) {
        String l = cf.readStringUntil('\n');
        l.trim();
        if (l.length() > 0) syncTotal++;
      }
      cf.close();
    }
  }

  syncSeq = 0;
  syncWaitingAck = false;
  syncRetries = 0;

  StaticJsonDocument<64> doc;
  doc["type"] = "header";
  doc["total"] = syncTotal;
  String out; serializeJson(doc, out);
  historySyncChar->setValue(out.c_str());
  historySyncChar->notify();

  if (syncTotal == 0) {
    StaticJsonDocument<32> doneDoc;
    doneDoc["type"] = "done";
    String doneOut; serializeJson(doneDoc, doneOut);
    historySyncChar->setValue(doneOut.c_str());
    historySyncChar->notify();
    syncActive = false;
    return;
  }

  syncFile = LittleFS.open("/history.csv", "r");
  if (!syncFile) {
    cancelHistorySync("storage_error", true);
    return;
  }
  syncReadPos = syncFile.size();
  syncActive = true;
}

void cancelHistorySync(const char* reason, bool sendError) {
  if (sendError && historySyncChar != nullptr) {
    StaticJsonDocument<64> doc;
    doc["type"] = "error";
    doc["reason"] = reason;
    String out; serializeJson(doc, out);
    historySyncChar->setValue(out.c_str());
    historySyncChar->notify();
  }
  syncActive = false;
  syncWaitingAck = false;
  if (syncFile) syncFile.close();
}

void serviceHistorySync() {
  if (!syncActive || !deviceConnected) return;

  if (!syncWaitingAck) {
    if (syncSeq < syncTotal) {
      sendNextSyncRecord();
    }
    return;
  }

  // waiting on an ack — check for timeout / retry
  if (millis() - syncSentMs >= SYNC_ACK_TIMEOUT_MS) {
    if (syncRetries == 0) {
      syncRetries = 1;
      resendCurrentSyncRecord(); // resend the same seq once
    } else {
      cancelHistorySync("timeout", true);
    }
  }
}

void initBLE() {
  BLEDevice::init(DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  vitalsChar = pService->createCharacteristic(
      CHAR_VITALS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  vitalsChar->addDescriptor(new BLE2902());
  vitalsChar->setValue("{\"bpm\":0,\"spo2\":0,\"finger\":false}");

  controlChar = pService->createCharacteristic(
      CHAR_CONTROL_UUID, BLECharacteristic::PROPERTY_WRITE);
  controlChar->setCallbacks(new ControlCallbacks());

  statusChar = pService->createCharacteristic(
      CHAR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusChar->addDescriptor(new BLE2902());
  statusChar->setValue("{\"active\":false,\"remaining\":0,\"done\":false}");

  historyChar = pService->createCharacteristic(
      CHAR_HISTORY_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  historyChar->setCallbacks(new HistoryCallbacks());

  historySyncChar = pService->createCharacteristic(
      CHAR_HISTORYSYNC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  historySyncChar->addDescriptor(new BLE2902());
  historySyncChar->setCallbacks(new HistorySyncCallbacks());

  profileChar = pService->createCharacteristic(
      CHAR_PROFILE_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  profileChar->setCallbacks(new ProfileCallbacks());
  loadProfileFromFS();

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started as: " DEVICE_NAME);
}