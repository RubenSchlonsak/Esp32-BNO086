#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 9
#define SCL_PIN 8

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t v;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // BNO Bootzeit
  delay(300);

  bool ok = false;
  for (int i = 0; i < 5 && !ok; ++i) {
    if (bno.begin_I2C(0x4B, &Wire) || bno.begin_I2C(0x4A, &Wire)) {
      ok = true;
      break;
    }
    Serial.println("# WARN: BNO086 not found, retry...");
    delay(200);
  }
  if (!ok) {
    Serial.println("# ERR: BNO086 not responding");
    while (1) delay(1000);
  }

  // Reports
  bno.enableReport(SH2_ACCELEROMETER,      10000);  // optional
  bno.enableReport(SH2_RAW_ACCELEROMETER,  10000);  // 100 Hz RAW

  // CSV-Header: Zeit in ms + RAW
  Serial.println("t_ms,raw_x,raw_y,raw_z");
}

void loop() {
  if (!bno.getSensorEvent(&v)) return;

  if (v.sensorId == SH2_RAW_ACCELEROMETER) {
    uint32_t t_ms = millis();  // Zeit seit Boot in ms
    Serial.printf("%lu,%d,%d,%d\n",
                  (unsigned long)t_ms,
                  (int)v.un.rawAccelerometer.x,
                  (int)v.un.rawAccelerometer.y,
                  (int)v.un.rawAccelerometer.z);
  }
}
