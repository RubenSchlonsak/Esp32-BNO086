#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "SparkFun_BMI270_Arduino_Library.h"
#include "Adafruit_MPR121.h"

// -------------------- PIN DEFINITIONEN --------------------
// Adafruit QT Py ESP32-S3 STEMMA QT Pins
#define STEMMA_SDA 41
#define STEMMA_SCL 40

// I2C Adressen
#define BMI270_ADDR BMI2_I2C_PRIM_ADDR // 0x68
#define MPR121_ADDR 0x5A               

// Anzahl Analog-Kanäle (0-3)
#define MPR_ANALOG_CHANNELS 4 

// -------------------- BLE Config --------------------
static const char* SERVICE_UUID      = "12345678-1234-1234-1234-123456789012";
static const char* DATA_CHAR_UUID    = "abcdef12-3456-789a-bcde-123456789abc";
static const char* CONTROL_CHAR_UUID = "12345678-1234-1234-1234-123456789013";

static const uint16_t MAGIC_V3    = 0xC1C1;
static const uint8_t  VER_V3      = 3;
static const uint8_t  REC_V3_SIZE = 24; 

// -------------------- Globale Objekte --------------------

BMI270 imu;
Adafruit_MPR121 cap = Adafruit_MPR121();

// Wir erstellen manuell das Wire Objekt für den zweiten Bus
TwoWire I2C_STEMMA = TwoWire(1); 

// Sampling
volatile uint32_t sampleIntervalUs = 1000000UL / 50; 
volatile uint32_t lastSampleUs     = 0;

// BLE
BLEServer* pServer           = nullptr;
BLECharacteristic* pDataChar = nullptr;
BLECharacteristic* pControlChar = nullptr;

volatile uint16_t usablePayloadBytes  = 20; 
volatile uint8_t  maxSamplesPerPacket = 1; 

struct PackedSample {
  uint16_t dt_ms;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  uint16_t touch_mask;              
  uint16_t mpr_vals[MPR_ANALOG_CHANNELS]; 
};

static const uint8_t MAX_BUF_SAMPLES = 20; 
PackedSample sampleBuf[MAX_BUF_SAMPLES];
uint8_t      sampleBufCount    = 0;
uint32_t     lastSampleTimeMs  = 0;

// -------------------- Helpers --------------------

static void recalcMaxSamples() {
  uint16_t payload = usablePayloadBytes;
  if (payload < 4 + REC_V3_SIZE) {
    maxSamplesPerPacket = 1; 
    return;
  }
  uint16_t maxCount = (payload - 4) / REC_V3_SIZE;
  if (maxCount > MAX_BUF_SAMPLES) maxCount = MAX_BUF_SAMPLES;
  if (maxCount == 0) maxCount = 1;
  maxSamplesPerPacket = (uint8_t)maxCount;
}

static void pushSample(const PackedSample& s) {
  if (sampleBufCount < MAX_BUF_SAMPLES) {
    sampleBuf[sampleBufCount++] = s;
  }
}

static void flushPacket() {
  if (sampleBufCount == 0 || !pDataChar) return;

  uint16_t len = 4 + sampleBufCount * REC_V3_SIZE;
  static uint8_t buf[512];

  buf[0] = (uint8_t)(MAGIC_V3 & 0xFF);
  buf[1] = (uint8_t)((MAGIC_V3 >> 8) & 0xFF);
  buf[2] = VER_V3;
  buf[3] = sampleBufCount;

  uint16_t offs = 4;
  for (uint8_t i = 0; i < sampleBufCount; ++i) {
    const PackedSample& s = sampleBuf[i];
    auto putInt16LE = [&](uint16_t v, uint16_t base) {
      buf[base + 0] = (uint8_t)(v & 0xFF);
      buf[base + 1] = (uint8_t)((v >> 8) & 0xFF);
    };

    putInt16LE(s.dt_ms, offs + 0);
    putInt16LE((uint16_t)s.ax, offs + 2);
    putInt16LE((uint16_t)s.ay, offs + 4);
    putInt16LE((uint16_t)s.az, offs + 6);
    putInt16LE((uint16_t)s.gx, offs + 8);
    putInt16LE((uint16_t)s.gy, offs + 10);
    putInt16LE((uint16_t)s.gz, offs + 12);
    putInt16LE(s.touch_mask, offs + 14);

    for(int k=0; k<MPR_ANALOG_CHANNELS; k++){
        putInt16LE(s.mpr_vals[k], offs + 16 + (k*2));
    }
    offs += REC_V3_SIZE;
  }

  pDataChar->setValue(buf, len);
  pDataChar->notify();
  sampleBufCount = 0;
}

// -------------------- Callbacks --------------------
class ControlCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    if (v.rfind("MTU:", 0) == 0) {
      uint16_t val = atoi(v.c_str() + 4);
      if (val >= 23) {
        usablePayloadBytes = val - 3; 
        recalcMaxSamples();
        Serial.printf("MTU: %u bytes -> %u samples\n", usablePayloadBytes, maxSamplesPerPacket);
      }
    }
    if (v.rfind("RATE:", 0) == 0) {
      uint16_t hz = atoi(v.c_str() + 5);
      if (hz >= 1) sampleIntervalUs = 1000000UL / hz;
    }
  }
};

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(3000); 
  Serial.println("\n--- ESP32-S3 Insole Start ---");

  // I2C Starten auf Pin 5 und 6
  I2C_STEMMA.begin(STEMMA_SDA, STEMMA_SCL, 400000);

  // --- HIER WAR DER FEHLER ---
  Serial.print("Pruefe BMI270... ");
  // SparkFun will das Objekt direkt, OHNE '&'
  if (imu.beginI2C(BMI270_ADDR, I2C_STEMMA) == BMI2_OK) {
    Serial.println("OK");
  } else {
    Serial.println("FEHLER!");
  }

  Serial.print("Pruefe MPR121... ");
  // Adafruit will einen Pointer, also MIT '&'
  if (cap.begin(MPR121_ADDR, &I2C_STEMMA)) {
    Serial.println("OK");
  } else {
    Serial.println("FEHLER!");
  }

  BLEDevice::init("ESP32-S3-Insole");
  pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(SERVICE_UUID);

  pDataChar = pService->createCharacteristic(DATA_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pDataChar->addDescriptor(new BLE2902());

  pControlChar = pService->createCharacteristic(CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pControlChar->setCallbacks(new ControlCallbacks());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE Bereit.");
  
  recalcMaxSamples();
  lastSampleUs = micros();
  lastSampleTimeMs = millis();
}

// -------------------- LOOP --------------------
void loop() {
  uint32_t nowUs = micros();

  if ((nowUs - lastSampleUs) >= sampleIntervalUs) {
    lastSampleUs = nowUs;

    imu.getSensorData();
    uint16_t touched = cap.touched();

    uint32_t nowMs = millis();
    uint16_t dt    = (uint16_t)(nowMs - lastSampleTimeMs);
    lastSampleTimeMs = nowMs;

    PackedSample s;
    s.dt_ms = dt;
    s.ax = (int16_t)roundf(imu.data.accelX * 1000.0f);
    s.ay = (int16_t)roundf(imu.data.accelY * 1000.0f);
    s.az = (int16_t)roundf(imu.data.accelZ * 1000.0f);
    s.gx = (int16_t)roundf(imu.data.gyroX * 1000.0f);
    s.gy = (int16_t)roundf(imu.data.gyroY * 1000.0f);
    s.gz = (int16_t)roundf(imu.data.gyroZ * 1000.0f);
    s.touch_mask = touched;

    for(int i=0; i<MPR_ANALOG_CHANNELS; i++) {
        s.mpr_vals[i] = cap.filteredData(i);
    }

    pushSample(s);

    if (sampleBufCount >= maxSamplesPerPacket) flushPacket();
  }

  static uint32_t lastFlushTime = 0;
  if (millis() - lastFlushTime > 30) {
      flushPacket();
      lastFlushTime = millis();
  }
  delay(1);
}