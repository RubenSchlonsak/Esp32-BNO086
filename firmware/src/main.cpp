#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN    9
#define SCL_PIN    8
#define BNO_ADDR   0x4B
#define REPORT_US  5000      // 200 Hz

Adafruit_BNO08x bno(-1);
static bool hostConnected = false;

void printHeader() {
  Serial.println("seq,t_ms,ax,ay,az");
}

void setup() {
  Serial.begin(921600);
  Serial.setTxTimeoutMs(20);        // verhindert Dauer-Blockieren
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!bno.begin_I2C(BNO_ADDR, &Wire)) {
    if (!bno.begin_I2C(0x4A, &Wire)) {
      // Kein endloses Printen, wenn kein Host dran
      for (;;) delay(1000);
    }
  }
  bno.enableReport(SH2_ACCELEROMETER, REPORT_US);
}

void loop() {
  // Verbindungstatus erkennen (CDC setzt das korrekt)
  bool now = (bool)Serial;
  if (now && !hostConnected) {
    // frisch verbunden -> Header einmal senden
    printHeader();
  }
  hostConnected = now;

  sh2_SensorValue_t v;
  while (bno.getSensorEvent(&v)) {
    if (v.sensorId != SH2_ACCELEROMETER) continue;

    // FIFO immer leeren, aber nur senden wenn Host verbunden
    if (hostConnected) {
      // nur senden, wenn genug Buffer frei (verhindert Blockieren)
      if (Serial.availableForWrite() >= 96) {
        static uint32_t seq=0;
        uint32_t t = millis();
        Serial.printf("%lu,%lu,%.7f,%.7f,%.7f\n",
                      (unsigned long)seq++,
                      (unsigned long)t,
                      v.un.accelerometer.x,
                      v.un.accelerometer.y,
                      v.un.accelerometer.z);
      }
    }
  }
}
