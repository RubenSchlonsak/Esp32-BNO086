#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>

#include "SparkFun_ISM330DHCX.h"

// ===== I2C pins (dein verifiziertes Mapping) =====
static constexpr int PIN_I2C_SDA = 9; // blau = SDA
static constexpr int PIN_I2C_SCL = 8; // gelb = SCL

// ===== IMU =====
SparkFun_ISM330DHCX imu;
sfe_ism_data_t a, g;

static bool imu_ok = false;
static volatile bool streaming = false;
static uint32_t fs_hz = 240;
static uint32_t seq = 0;
static uint64_t next_tick_us = 0;

// ===== BLE UUIDs (NUS-style) =====
static const char* SVC_UUID   = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* CTRL_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* DATA_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static NimBLECharacteristic* ctrlChar = nullptr;
static NimBLECharacteristic* dataChar = nullptr;

#pragma pack(push, 1)
struct DataPkt {
  uint32_t seq;
  uint64_t dev_time_us;
  int16_t ax_mg, ay_mg, az_mg;
  int16_t gx_mdps, gy_mdps, gz_mdps;
};
#pragma pack(pop)

static_assert(sizeof(DataPkt) == 24, "Packet size must be 24 bytes");

// ===== helpers =====
static int16_t clamp_i16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  - 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  (no devices found)");
}

// SparkFun-Lib kann je nach Version imu.begin() oder imu.begin(Wire) haben.
// Diese Helper wählen compile-time die vorhandene Signatur.
template <typename T>
static auto imuBeginImpl(T& dev, TwoWire& w, int) -> decltype(dev.begin(w), bool()) {
  return dev.begin(w);
}
template <typename T>
static auto imuBeginImpl(T& dev, TwoWire&, long) -> decltype(dev.begin(), bool()) {
  return dev.begin();
}
static bool imuBeginAuto(SparkFun_ISM330DHCX& dev, TwoWire& w) {
  return imuBeginImpl(dev, w, 0);
}

static auto mapAccelOdr(uint32_t hz) -> decltype(ISM330DHCX_XL_ODR_12Hz5) {
  if (hz <= 12)   return ISM330DHCX_XL_ODR_12Hz5;
  if (hz <= 26)   return ISM330DHCX_XL_ODR_26Hz;
  if (hz <= 52)   return ISM330DHCX_XL_ODR_52Hz;
  if (hz <= 104)  return ISM330DHCX_XL_ODR_104Hz;
  if (hz <= 208)  return ISM330DHCX_XL_ODR_208Hz;
  if (hz <= 416)  return ISM330DHCX_XL_ODR_416Hz;
  if (hz <= 833)  return ISM330DHCX_XL_ODR_833Hz;
  return ISM330DHCX_XL_ODR_1666Hz;
}

static auto mapGyroOdr(uint32_t hz) -> decltype(ISM330DHCX_GY_ODR_12Hz5) {
  if (hz <= 12)   return ISM330DHCX_GY_ODR_12Hz5;
  if (hz <= 26)   return ISM330DHCX_GY_ODR_26Hz;
  if (hz <= 52)   return ISM330DHCX_GY_ODR_52Hz;
  if (hz <= 104)  return ISM330DHCX_GY_ODR_104Hz;
  if (hz <= 208)  return ISM330DHCX_GY_ODR_208Hz;
  if (hz <= 416)  return ISM330DHCX_GY_ODR_416Hz;
  if (hz <= 833)  return ISM330DHCX_GY_ODR_833Hz;
  return ISM330DHCX_GY_ODR_1666Hz;
}

static void applySampling(uint32_t hz) {
  fs_hz = hz;
  if (imu_ok) {
    imu.setAccelDataRate(mapAccelOdr(hz));
    imu.setGyroDataRate(mapGyroOdr(hz));
  }
  next_tick_us = (uint64_t)esp_timer_get_time();
}

static void startStream() {
  seq = 0;
  streaming = true;
  next_tick_us = (uint64_t)esp_timer_get_time();
}

static void stopStream() {
  streaming = false;
}

// ===== BLE callbacks =====
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string s = c->getValue();
    String cmd = String(s.c_str());
    cmd.trim();

    if (cmd.equalsIgnoreCase("PING")) {
      c->setValue("PONG");
      return;
    }
    if (cmd.equalsIgnoreCase("START")) {
      if (!imu_ok) { c->setValue("ERR IMU not ready"); return; }
      startStream();
      c->setValue("OK START");
      return;
    }
    if (cmd.equalsIgnoreCase("STOP")) {
      stopStream();
      c->setValue("OK STOP");
      return;
    }
    if (cmd.startsWith("SR=") || cmd.startsWith("sr=")) {
      uint32_t hz = (uint32_t)cmd.substring(3).toInt();
      if (hz < 1 || hz > 960) {
        c->setValue("ERR SR range 1..960");
        return;
      }
      applySampling(hz);
      c->setValue((String("OK SR=") + String(fs_hz)).c_str());
      return;
    }

    c->setValue("ERR unknown");
  }
};

static void setupBle() {
  NimBLEDevice::init("ESP32S3-ISM330DHCX");
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* svc = server->createService(SVC_UUID);

  ctrlChar = svc->createCharacteristic(
    CTRL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ
  );
  ctrlChar->setCallbacks(new CtrlCallbacks());
  ctrlChar->setValue("BOOT");

  dataChar = svc->createCharacteristic(
    DATA_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName("ESP32S3-ISM330DHCX");
  adv->addServiceUUID(SVC_UUID);
  adv->setAppearance(0x0341);
  adv->start();
}

static void setupImu() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); // für 416Hz+ sinnvoll; du hattest Bus stabil
  Serial.printf("I2C: SDA=%d SCL=%d @400k\n", PIN_I2C_SDA, PIN_I2C_SCL);
  i2cScan();

  // SA0=VDD => 0x6B (Hookup Guide)
  // Library findet das selbst; du hast 0x6B bereits gescannt.
  bool ok = imuBeginAuto(imu, Wire);
  if (!ok) {
    Serial.println("IMU: begin failed");
    imu_ok = false;
    if (ctrlChar) ctrlChar->setValue("ERR IMU.begin");
    return;
  }

  imu_ok = true;
  Serial.println("IMU: begin OK");

  // Full-scale settings (SparkFun API: setAccelFullScale / setGyroFullScale)
  imu.setAccelFullScale(ISM_2g);
  imu.setGyroFullScale(ISM_500dps);

  // Data rates
  applySampling(fs_hz);

  if (ctrlChar) ctrlChar->setValue("OK READY");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BOOT: start");

  setupBle();   // BLE immer zuerst
  setupImu();

  Serial.println("BOOT: done");
}

void loop() {
  // Heartbeat
  static uint32_t last_hb = 0;
  if (millis() - last_hb > 1000) {
    last_hb = millis();
    Serial.printf("HB: imu_ok=%d streaming=%d sr=%lu\n",
                  imu_ok ? 1 : 0,
                  streaming ? 1 : 0,
                  (unsigned long)fs_hz);
  }

  if (!streaming || !imu_ok) {
    delay(10);
    return;
  }

  const uint64_t now_us = (uint64_t)esp_timer_get_time();
  const uint64_t period_us = (uint64_t)(1000000ULL / (uint64_t)max<uint32_t>(1, fs_hz));

  if (now_us < next_tick_us) {
    delayMicroseconds((uint32_t)min<uint64_t>(2000ULL, next_tick_us - now_us));
    return;
  }
  next_tick_us += period_us;

  // SparkFun API: getAccel / getGyro fill sfe_ism_data_t (float x/y/z)
  if (!imu.checkStatus()) return;

  imu.getAccel(&a);
  imu.getGyro(&g);

  // Typical units from SparkFun examples/community are g and dps; convert to mg/mdps.
  long ax_mg = lround(a.xData * 1000.0f);
  long ay_mg = lround(a.yData * 1000.0f);
  long az_mg = lround(a.zData * 1000.0f);

  long gx_mdps = lround(g.xData * 1000.0f);
  long gy_mdps = lround(g.yData * 1000.0f);
  long gz_mdps = lround(g.zData * 1000.0f);

  DataPkt pkt;
  pkt.seq = seq++;
  pkt.dev_time_us = now_us;
  pkt.ax_mg = clamp_i16(ax_mg);
  pkt.ay_mg = clamp_i16(ay_mg);
  pkt.az_mg = clamp_i16(az_mg);
  pkt.gx_mdps = clamp_i16(gx_mdps);
  pkt.gy_mdps = clamp_i16(gy_mdps);
  pkt.gz_mdps = clamp_i16(gz_mdps);

  dataChar->setValue((uint8_t*)&pkt, sizeof(pkt));
  dataChar->notify();
}
