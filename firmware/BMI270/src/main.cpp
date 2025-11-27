#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_LSM6DSV16X.h"

// I2C-Pins ESP32-S3 (anpassen falls nötig)
#define SDA_PIN 41
#define SCL_PIN 40

SparkFun_LSM6DSV16X imu;
sfe_lsm_data_t accelData;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // I2C mit deinen Pins starten
  Wire.begin(SDA_PIN, SCL_PIN);

  // IMU initialisieren
  if (!imu.begin()) {
    Serial.println("IMU not found!");
    while (1) {
      delay(100);
    }
  }

  // Reset auf Default
  imu.deviceReset();
  while (!imu.getDeviceReset()) {
    delay(1);
  }

  Serial.println("Board has been reset.");
  imu.enableBlockDataUpdate();

  // ±2 g, ca. 240 Hz
  imu.setAccelFullScale(LSM6DSV16X_2g);
  imu.setAccelDataRate(LSM6DSV16X_ODR_AT_240Hz);

  // CSV-Header
  Serial.println("TimeUs,AccelX,AccelY,AccelZ");

  Serial.println("Ready.");
}

void loop() {
  // Polling mit fester Periode ~240 Hz
  static uint32_t lastReadUs = 0;
  const uint32_t periodUs = 1000000UL / 240UL; // ~4167 µs

  uint32_t now = micros();
  if (now - lastReadUs < periodUs) {
    return; // noch nicht wieder dran
  }
  lastReadUs = now;

  // Beschleunigung holen
  imu.getAccel(&accelData);

  float ax = accelData.xData;
  float ay = accelData.yData;
  float az = accelData.zData;
  uint32_t t = micros();

  Serial.print(t);
  Serial.print(",");
  Serial.print(ax, 6);
  Serial.print(",");
  Serial.print(ay, 6);
  Serial.print(",");
  Serial.println(az, 6);
}
