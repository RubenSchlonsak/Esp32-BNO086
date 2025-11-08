#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "SparkFun_BMI270_Arduino_Library.h"

// I2C Pins
#define SDA_PIN 9
#define SCL_PIN 8

// BLE UUIDs – passend zum Python-Script
static const char* SERVICE_UUID      = "12345678-1234-1234-1234-123456789012";
static const char* DATA_CHAR_UUID    = "abcdef12-3456-789a-bcde-123456789abc";
static const char* CONTROL_CHAR_UUID = "12345678-1234-1234-1234-123456789013";

// Protokoll (V2)
static const uint16_t MAGIC_V2    = 0xB0B0;
static const uint8_t  VER_V2      = 2;
static const uint8_t  REC_V2_SIZE = 14; // dt(u16) + 6 * int16

// BMI270
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

// Sampling
volatile uint32_t sampleIntervalUs = 1000000UL / 200; // 200 Hz
volatile uint32_t lastSampleUs     = 0;

// BLE
BLEServer*         pServer      = nullptr;
BLECharacteristic* pDataChar    = nullptr;
BLECharacteristic* pControlChar = nullptr;

volatile uint16_t usablePayloadBytes  = 20;
volatile uint8_t  maxSamplesPerPacket = 1;

// Sample-Puffer
struct PackedSample {
  uint16_t dt_ms;
  int16_t  ax;
  int16_t  ay;
  int16_t  az;
  int16_t  gx;
  int16_t  gy;
  int16_t  gz;
};

static const uint8_t MAX_BUF_SAMPLES = 32;
PackedSample sampleBuf[MAX_BUF_SAMPLES];
uint8_t      sampleBufCount    = 0;
uint32_t     lastSampleTimeMs  = 0;

// -------------------- Helpers --------------------
static void recalcMaxSamples() {
  uint16_t payload = usablePayloadBytes;
  if (payload < 4 + REC_V2_SIZE) {
    maxSamplesPerPacket = 1;
    return;
  }
  uint16_t maxCount = (payload - 4) / REC_V2_SIZE;
  if (maxCount == 0) maxCount = 1;
  if (maxCount > MAX_BUF_SAMPLES) maxCount = MAX_BUF_SAMPLES;
  maxSamplesPerPacket = (uint8_t)maxCount;
}

static void pushSample(const PackedSample& s) {
  if (sampleBufCount < maxSamplesPerPacket && sampleBufCount < MAX_BUF_SAMPLES) {
    sampleBuf[sampleBufCount++] = s;
  }
}

static void flushPacket() {
  if (sampleBufCount == 0 || !pDataChar) return;

  uint16_t len = 4 + sampleBufCount * REC_V2_SIZE;
  static uint8_t buf[4 + MAX_BUF_SAMPLES * REC_V2_SIZE];

  buf[0] = (uint8_t)(MAGIC_V2 & 0xFF);
  buf[1] = (uint8_t)((MAGIC_V2 >> 8) & 0xFF);
  buf[2] = VER_V2;
  buf[3] = sampleBufCount;

  uint16_t offs = 4;
  for (uint8_t i = 0; i < sampleBufCount; ++i) {
    const PackedSample& s = sampleBuf[i];

    buf[offs + 0] = (uint8_t)(s.dt_ms & 0xFF);
    buf[offs + 1] = (uint8_t)((s.dt_ms >> 8) & 0xFF);

    auto putInt16LE = [&](int16_t v, uint16_t base) {
      buf[base + 0] = (uint8_t)(v & 0xFF);
      buf[base + 1] = (uint8_t)((v >> 8) & 0xFF);
    };

    putInt16LE(s.ax, offs + 2);
    putInt16LE(s.ay, offs + 4);
    putInt16LE(s.az, offs + 6);
    putInt16LE(s.gx, offs + 8);
    putInt16LE(s.gy, offs + 10);
    putInt16LE(s.gz, offs + 12);

    offs += REC_V2_SIZE;
  }

  pDataChar->setValue(buf, len);
  pDataChar->notify();

  sampleBufCount = 0;
}

// -------------------- Control-Callbacks --------------------
class ControlCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    if (v.rfind("MTU:", 0) == 0) {
      uint16_t val = atoi(v.c_str() + 4);
      if (val >= 23 && val <= 247) {
        usablePayloadBytes = val;
        recalcMaxSamples();
        Serial.printf("# BLE: usable payload=%u, maxSamples=%u\n",
                      usablePayloadBytes, maxSamplesPerPacket);
      }
      return;
    }

    if (v.rfind("RATE:", 0) == 0) {
      uint16_t hz = atoi(v.c_str() + 5);
      if (hz >= 1 && hz <= 1000) {
        sampleIntervalUs = 1000000UL / hz;
        Serial.printf("# BLE: sample rate=%u Hz\n", hz);
      }
      return;
    }

    Serial.printf("# BLE: unknown cmd '%s'\n", v.c_str());
  }
};

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  while (imu.beginI2C(i2cAddress) != BMI2_OK) {
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
    delay(1000);
  }
  Serial.println("# BMI270 connected");

  // BLE Initialisierung
  BLEDevice::init("ESP32-IMU");

  // Optional: Sendeleistung (je nach Core ggf. ESP_PWR_LVL_P9 o.ä.)
  // BLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pDataChar = pService->createCharacteristic(
      DATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  // Descriptor für Notify (iOS & Co.)
  pDataChar->addDescriptor(new BLE2902());

  pControlChar = pService->createCharacteristic(
      CONTROL_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  pControlChar->setCallbacks(new ControlCallbacks());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // optional
  pAdv->setMaxPreferred(0x12);  // optional
  BLEDevice::startAdvertising();

  Serial.println("# BLE advertising as ESP32-IMU");

  recalcMaxSamples();
  lastSampleUs     = micros();
  lastSampleTimeMs = millis();
}

// -------------------- Loop --------------------
void loop() {
  uint32_t nowUs = micros();
  if ((nowUs - lastSampleUs) >= sampleIntervalUs) {
    lastSampleUs = nowUs;

    imu.getSensorData();

    float ax_g = imu.data.accelX;
    float ay_g = imu.data.accelY;
    float az_g = imu.data.accelZ;

    float gx_dps = imu.data.gyroX;
    float gy_dps = imu.data.gyroY;
    float gz_dps = imu.data.gyroZ;

    uint32_t nowMs = millis();
    uint16_t dt    = (uint16_t)(nowMs - lastSampleTimeMs);
    lastSampleTimeMs = nowMs;

    PackedSample s;
    s.dt_ms = dt;
    s.ax    = (int16_t)roundf(ax_g   * 1000.0f);
    s.ay    = (int16_t)roundf(ay_g   * 1000.0f);
    s.az    = (int16_t)roundf(az_g   * 1000.0f);
    s.gx    = (int16_t)roundf(gx_dps * 1000.0f);
    s.gy    = (int16_t)roundf(gy_dps * 1000.0f);
    s.gz    = (int16_t)roundf(gz_dps * 1000.0f);

    pushSample(s);

    if (sampleBufCount >= maxSamplesPerPacket) {
      flushPacket();
    }
  }

  static uint32_t lastFlushMs = 0;
  uint32_t nowMs = millis();
  if ((nowMs - lastFlushMs) >= 20) {
    flushPacket();
    lastFlushMs = nowMs;
  }

  delay(1);
}
