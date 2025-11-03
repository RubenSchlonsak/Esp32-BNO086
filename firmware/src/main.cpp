#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 9
#define SCL_PIN 8

Adafruit_BNO08x bno(-1);

void setup() {
  Serial.begin(921600);
  while(!Serial) delay(10);
  
  Serial.println("\n\n# BNO086 Test Start");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!bno.begin_I2C(0x4B, &Wire)) {
    if (!bno.begin_I2C(0x4A, &Wire)) {
      Serial.println("# FAIL: Sensor not found!");
      while(1) delay(100);
    }
  }
  
  Serial.println("# OK: Sensor found");
  
  // Versuche verschiedene Sensoren
  Serial.println("# Trying ACCELEROMETER...");
  bno.enableReport(SH2_ACCELEROMETER, 10000); // 100Hz
  
  Serial.println("# Trying GYROSCOPE...");
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  
  Serial.println("# Setup done\n");
}

void loop() {
  sh2_SensorValue_t v;
  
  if (bno.getSensorEvent(&v)) {
    Serial.print("SensorID: ");
    Serial.print(v.sensorId);
    Serial.print(" | ");
    
    switch(v.sensorId) {
      case SH2_ACCELEROMETER:
        Serial.printf("Accel: %.3f, %.3f, %.3f\n",
                     v.un.accelerometer.x,
                     v.un.accelerometer.y,
                     v.un.accelerometer.z);
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        Serial.printf("Gyro: %.3f, %.3f, %.3f\n",
                     v.un.gyroscope.x,
                     v.un.gyroscope.y,
                     v.un.gyroscope.z);
        break;
        
      default:
        Serial.println("Unknown sensor");
        break;
    }
  }
  
  delay(1); // Kleine Pause
}