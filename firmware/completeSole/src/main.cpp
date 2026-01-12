#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_LSM6DSV16X.h"
#include <NimBLEDevice.h>

// =======================================================
// USER CONFIG
// =======================================================
static const uint32_t SAMPLE_HZ   = 200;
static const uint32_t SERIAL_BAUD = 921600;

static const int PIN_SDA = 41;
static const int PIN_SCL = 40;
static const uint32_t I2C_FREQ_HZ = 400000;

static const char* DEVICE_NAME = "ESP32-Insole";

// Match Flutter UUIDs
static const char* SERVICE_UUID        = "12345678-1234-1234-1234-123456789012";
static const char* CHARACTERISTIC_UUID = "abcdef12-3456-789a-bcde-123456789abc";

// Queue sizing: 200 Hz -> 200 lines/s. 400 entries ~2 s buffer
static const int QUEUE_DEPTH = 400;

// Avoid LINE_MAX macro collision
static const size_t CSV_LINE_MAX = 200; // bumped slightly for seq field

// =======================================================
static const uint32_t SAMPLE_PERIOD_MS = 1000UL / SAMPLE_HZ;

// =======================================================
// IMU
// =======================================================
SparkFun_LSM6DSV16X imu;
sfe_lsm_data_t accelData{};
sfe_lsm_data_t gyroData{};

// =======================================================
// MPR121 (6 channels only)
// =======================================================
static const uint8_t MPR121_ADDR = 0x5A;
static const uint8_t MPR121_FILTDATA_0L = 0x04;
static const uint8_t MPR121_TOUCHTH_0   = 0x41;
static const uint8_t MPR121_RELEASETH_0 = 0x42;
static const uint8_t MPR121_CONFIG1     = 0x5C;
static const uint8_t MPR121_CONFIG2     = 0x5D;
static const uint8_t MPR121_ECR         = 0x5E;
static const uint8_t MPR121_AUTOCONFIG0 = 0x7B;
static const uint8_t MPR121_UPLIMIT     = 0x7D;
static const uint8_t MPR121_LOWLIMIT    = 0x7E;
static const uint8_t MPR121_TARGETLIMIT = 0x7F;
static const uint8_t MPR121_SOFTRESET   = 0x80;

float cap[6] = {0, 0, 0, 0, 0, 0};

// =======================================================
// Sequence counter (monotonic sample id)
// =======================================================
static uint32_t gSeq = 0;

// =======================================================
// I2C helpers
// =======================================================
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// =======================================================
// MPR121 init — 6 electrodes only
// =======================================================
static bool mpr121Init6ch(uint8_t touchTh = 12, uint8_t releaseTh = 6) {
  delay(10);
  if (!i2cWrite8(MPR121_ADDR, MPR121_SOFTRESET, 0x63)) return false;
  delay(2);

  uint8_t v;
  if (!i2cRead(MPR121_ADDR, MPR121_CONFIG1, &v, 1) || v != 0x10) return false;
  if (!i2cRead(MPR121_ADDR, MPR121_CONFIG2, &v, 1) || v != 0x24) return false;

  i2cWrite8(MPR121_ADDR, MPR121_AUTOCONFIG0, 0x07);
  i2cWrite8(MPR121_ADDR, MPR121_UPLIMIT, 200);
  i2cWrite8(MPR121_ADDR, MPR121_TARGETLIMIT, 180);
  i2cWrite8(MPR121_ADDR, MPR121_LOWLIMIT, 130);

  i2cWrite8(MPR121_ADDR, MPR121_CONFIG2, 0x20);

  for (int ch = 0; ch < 6; ch++) {
    i2cWrite8(MPR121_ADDR, MPR121_TOUCHTH_0 + 2 * ch, touchTh);
    i2cWrite8(MPR121_ADDR, MPR121_RELEASETH_0 + 2 * ch, releaseTh);
  }

  i2cWrite8(MPR121_ADDR, MPR121_ECR, 0x00);
  i2cWrite8(MPR121_ADDR, MPR121_ECR, 0x06); // enable 6 electrodes
  return true;
}

static void mpr121Read6(float out[6]) {
  uint8_t buf[0x2A];
  if (!i2cRead(MPR121_ADDR, 0x00, buf, sizeof(buf))) return;

  for (int i = 0; i < 6; i++) {
    int l = MPR121_FILTDATA_0L + 2 * i;
    int h = l + 1;
    out[i] = (float)(buf[l] | ((buf[h] & 0x03) << 8));
  }
}

// =======================================================
// IMU init (tuned for micro-vibration / HR-like sensing)
// =======================================================
static bool imuInit() {
  if (!imu.begin()) return false;

  imu.deviceReset();
  while (!imu.getDeviceReset()) delay(1);

  imu.enableBlockDataUpdate();
  imu.enableFilterSettling();

  imu.setAccelDataRate(LSM6DSV16X_ODR_AT_240Hz);
  imu.setGyroDataRate(LSM6DSV16X_ODR_AT_240Hz);

  imu.setAccelFullScale(LSM6DSV16X_2g);
  imu.setGyroFullScale(LSM6DSV16X_250dps);

  return true;
}

// =======================================================
// BLE globals
// =======================================================
static NimBLEServer*         gServer = nullptr;
static NimBLECharacteristic* gChar   = nullptr;

static volatile bool gConnected  = false;
static volatile bool gSubscribed = false;

// Queue item
struct LineItem {
  uint16_t len;
  char data[CSV_LINE_MAX];
};

static QueueHandle_t gLineQueue;

// =======================================================
// BLE callbacks (version-tolerant)
// =======================================================
class ServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* s) { gConnected = true; }
  void onDisconnect(NimBLEServer* s) {
    gConnected = false;
    gSubscribed = false;
    NimBLEDevice::startAdvertising();
  }

  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) { gConnected = true; }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) {
    gConnected = false;
    gSubscribed = false;
    NimBLEDevice::startAdvertising();
  }
};

class CharCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& connInfo, uint16_t subValue) {
    gSubscribed = (subValue != 0);
  }
};

// =======================================================
// Advertising: name guaranteed in Scan Response
// =======================================================
static void bleInit() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);

  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(new ServerCallbacks());

  NimBLEService* service = gServer->createService(SERVICE_UUID);

  gChar = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );
  gChar->setCallbacks(new CharCallbacks());
  gChar->createDescriptor("2902"); // CCCD

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.addServiceUUID(SERVICE_UUID);
  adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);
  adv->setScanResponseData(scanData);

  adv->start();
}

// =======================================================
// CSV header and status lines
// =======================================================
static const char* CSV_HEADER =
  "seq,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,cap0,cap1,cap2,cap3,cap4,cap5\n";

// =======================================================
// Notify with MTU-aware chunking
// =======================================================
static void notifyBytesChunked(const uint8_t* data, size_t len) {
  if (!gChar) return;

  const uint16_t mtu = NimBLEDevice::getMTU();
  const size_t maxPayload = (mtu > 3) ? (mtu - 3) : 20;

  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > maxPayload) n = maxPayload;

    gChar->setValue(data + off, n);
    gChar->notify();

    off += n;
    vTaskDelay(1);
  }
}

static void queueLine(const char* s) {
  if (!gLineQueue) return;
  LineItem item{};
  size_t n = strnlen(s, CSV_LINE_MAX - 1);
  memcpy(item.data, s, n);
  item.data[n] = '\0';
  item.len = (uint16_t)n;

  (void)xQueueSend(gLineQueue, &item, 0);
}

// =======================================================
// Task: sensor sampling -> queue
// =======================================================
static volatile bool gImuOk = false;
static volatile bool gMprOk = false;

static void SensorTask(void* arg) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (gImuOk && imu.checkStatus()) {
      imu.getAccel(&accelData);
      imu.getGyro(&gyroData);
    } else {
      accelData.xData = accelData.yData = accelData.zData = NAN;
      gyroData.xData  = gyroData.yData  = gyroData.zData  = NAN;
    }

    if (gMprOk) {
      mpr121Read6(cap);
    } else {
      for (int i = 0; i < 6; i++) cap[i] = NAN;
    }

    const uint32_t seq = gSeq++;

    LineItem item{};
    item.len = (uint16_t)snprintf(
      item.data, CSV_LINE_MAX,
      "%lu,%lu,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
      (unsigned long)seq,
      (unsigned long)millis(),
      accelData.xData, accelData.yData, accelData.zData,
      gyroData.xData,  gyroData.yData,  gyroData.zData,
      cap[0], cap[1], cap[2], cap[3], cap[4], cap[5]
    );

    if (item.len >= CSV_LINE_MAX) {
      item.len = CSV_LINE_MAX - 1;
      item.data[item.len] = '\n';
    }

    (void)xQueueSend(gLineQueue, &item, 0);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

// =======================================================
// Task: BLE notify consumer
// =======================================================
static void BleNotifyTask(void* arg) {
  bool headerSent = false;

  while (true) {
    if (!(gConnected && gSubscribed)) {
      headerSent = false;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (!headerSent) {
      notifyBytesChunked((const uint8_t*)CSV_HEADER, strlen(CSV_HEADER));
      headerSent = true;
    }

    LineItem item;
    if (xQueueReceive(gLineQueue, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
      notifyBytesChunked((const uint8_t*)item.data, item.len);
    }
  }
}

// =======================================================
// Arduino setup/loop
// =======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("BOOT,OK");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_FREQ_HZ);

  gImuOk = imuInit();
  gMprOk = mpr121Init6ch();

  Serial.print("SENSOR,IMU,");   Serial.println(gImuOk ? "OK" : "FAIL");
  Serial.print("SENSOR,MPR121,");Serial.println(gMprOk ? "OK" : "FAIL");

  gLineQueue = xQueueCreate(QUEUE_DEPTH, sizeof(LineItem));
  if (!gLineQueue) {
    Serial.println("ERR,QUEUE");
  }

  bleInit();
  Serial.print("BLE,ADV,NAME,"); Serial.println(DEVICE_NAME);

  queueLine("STATUS,BOOT,OK\n");
  if (!gImuOk) queueLine("STATUS,WARN,IMU_FAIL\n");
  if (!gMprOk) queueLine("STATUS,WARN,MPR121_FAIL\n");

  if (gLineQueue) {
    xTaskCreatePinnedToCore(SensorTask,    "SensorTask",    4096, nullptr, 2, nullptr, 1);
  }
  xTaskCreatePinnedToCore(BleNotifyTask, "BleNotifyTask", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
