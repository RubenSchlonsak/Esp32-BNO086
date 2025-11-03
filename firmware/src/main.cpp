#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 9
#define SCL_PIN 8

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t v;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n\n# BNO086 RAW Test Start");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno.begin_I2C(0x4B, &Wire)) {
    if (!bno.begin_I2C(0x4A, &Wire)) {
      Serial.println("# FAIL: Sensor not found!");
      while (1) delay(100);
    }
  }
  Serial.println("# OK: Sensor found");

  // Workaround: mindestens EIN „normaler“ Report + RAW aktivieren
  if (!bno.enableReport(SH2_ACCELEROMETER, 10000)) {
    Serial.println("# WARN: could not enable ACCEL");
  }
  if (!bno.enableReport(SH2_RAW_ACCELEROMETER, 10000)) {
    Serial.println("# WARN: could not enable RAW ACCEL");
  }

  Serial.println("# Setup done\n");
}

void loop() {
  if (!bno.getSensorEvent(&v)) {
    delay(1);
    return;
  }

  if (bno.wasReset()) {
    Serial.println("# NOTE: IMU reset, re-enabling reports");
    bno.enableReport(SH2_ACCELEROMETER, 10000);
    bno.enableReport(SH2_RAW_ACCELEROMETER, 10000);
  }

  Serial.print("SensorID: ");
  Serial.print(v.sensorId);
  Serial.print(" | ");

  switch (v.sensorId) {
    case SH2_ACCELEROMETER:
      //Serial.printf("Accel: %.3f, %.3f, %.3f\n",
                    //v.un.accelerometer.x,
                   // v.un.accelerometer.y,
                   // v.un.accelerometer.z);
      break;

    case SH2_RAW_ACCELEROMETER:
      // RAW = int16_t Counts → %d oder nach float casten
      Serial.printf("Raw: %d, %d, %d (cts)\n",
                    (int)v.un.rawAccelerometer.x,
                    (int)v.un.rawAccelerometer.y,
                    (int)v.un.rawAccelerometer.z);
      break;

    default:
      Serial.println();
      break;
  }
}
