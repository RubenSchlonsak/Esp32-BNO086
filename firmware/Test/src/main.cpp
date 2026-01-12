// ESP32 Battery Monitor mit BLE
// Sendet Batteriestatus per Bluetooth Low Energy

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const int BATTERY_PIN = 9;  // GPIO9 für Batteriemessung
const int UPDATE_INTERVAL = 2000;  // Aktualisierung alle 2 Sekunden

// Spannungsschwellen
const float USB_THRESHOLD = 4.3;
const float BATTERY_FULL = 4.2;
const float BATTERY_CRITICAL = 3.0;

// Voltage Divider
const float VOLTAGE_DIVIDER_RATIO = 2.0;
const float ADC_REFERENCE = 3.3;
const int ADC_RESOLUTION = 4095;

// BLE UUIDs (Standard Battery Service)
#define SERVICE_UUID           "0000180F-0000-1000-8000-00805F9B34FB"  // Battery Service
#define BATTERY_LEVEL_UUID     "00002A19-0000-1000-8000-00805F9B34FB"  // Battery Level
#define VOLTAGE_UUID           "00002A58-0000-1000-8000-00805F9B34FB"  // Custom für Spannung
#define STATUS_UUID            "00002A57-0000-1000-8000-00805F9B34FB"  // Custom für Status

BLEServer* pServer = NULL;
BLECharacteristic* pBatteryLevelChar = NULL;
BLECharacteristic* pVoltageChar = NULL;
BLECharacteristic* pStatusChar = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client verbunden!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client getrennt!");
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=================================");
  Serial.println("ESP32 Battery Monitor mit BLE");
  Serial.println("=================================");
  
  // ADC konfigurieren
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);
  
  // BLE initialisieren
  Serial.println("Starte BLE...");
  BLEDevice::init("ESP32 Battery");
  
  // BLE Server erstellen
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Battery Service erstellen
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Battery Level Characteristic (Standard)
  pBatteryLevelChar = pService->createCharacteristic(
                        BATTERY_LEVEL_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pBatteryLevelChar->addDescriptor(new BLE2902());
  
  // Voltage Characteristic (Custom)
  pVoltageChar = pService->createCharacteristic(
                   VOLTAGE_UUID,
                   BLECharacteristic::PROPERTY_READ |
                   BLECharacteristic::PROPERTY_NOTIFY
                 );
  pVoltageChar->addDescriptor(new BLE2902());
  
  // Status Characteristic (Custom)
  pStatusChar = pService->createCharacteristic(
                  STATUS_UUID,
                  BLECharacteristic::PROPERTY_READ |
                  BLECharacteristic::PROPERTY_NOTIFY
                );
  pStatusChar->addDescriptor(new BLE2902());
  
  // Service starten
  pService->start();
  
  // Advertising starten
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE gestartet!");
  Serial.println("Gerätename: ESP32 Battery");
  Serial.println("Verbinde mit einer BLE App (z.B. nRF Connect)");
  Serial.println("=================================");
  Serial.println();
}


float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(5);
  }
  int rawValue = sum / 10;
  
  float measuredVoltage = (rawValue / (float)ADC_RESOLUTION) * ADC_REFERENCE;
  float actualVoltage = measuredVoltage * VOLTAGE_DIVIDER_RATIO;
  
  return actualVoltage;
}

uint8_t calculateBatteryPercent(float voltage) {
  if (voltage >= BATTERY_FULL) {
    return 100;
  } else if (voltage <= BATTERY_CRITICAL) {
    return 0;
  }
  
  float percent = ((voltage - BATTERY_CRITICAL) / (BATTERY_FULL - BATTERY_CRITICAL)) * 100.0;
  return (uint8_t)constrain(percent, 0, 100);
}

String getStatus(float voltage) {
  if (voltage > USB_THRESHOLD) {
    return "USB Connected";
  } else if (voltage >= BATTERY_FULL) {
    return "Battery Full";
  } else if (voltage >= 3.8) {
    return "Battery Good";
  } else if (voltage >= 3.4) {
    return "Battery Low";
  } else if (voltage >= BATTERY_CRITICAL) {
    return "Battery Critical";
  } else {
    return "No Power";
  }
}

void loop() {
  // Batterie messen
  float voltage = readBatteryVoltage();
  uint8_t batteryPercent = calculateBatteryPercent(voltage);
  String status = getStatus(voltage);
  
  // Serial ausgeben
  Serial.println("--- Battery Status ---");
  Serial.print("Spannung: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  Serial.print("Ladezustand: ");
  Serial.print(batteryPercent);
  Serial.println(" %");
  Serial.print("Status: ");
  Serial.println(status);
  
  // BLE aktualisieren wenn verbunden
  if (deviceConnected) {
    // Battery Level (0-100%)
    pBatteryLevelChar->setValue(&batteryPercent, 1);
    pBatteryLevelChar->notify();
    
    // Spannung (als String, z.B. "3.85V")
    String voltageStr = String(voltage, 2) + "V";
    pVoltageChar->setValue(voltageStr.c_str());
    pVoltageChar->notify();
    
    // Status (USB/Battery/Low etc.)
    pStatusChar->setValue(status.c_str());
    pStatusChar->notify();
    
    Serial.println("✓ BLE Daten gesendet");
  } else {
    Serial.println("○ Warte auf BLE Verbindung...");
  }
  
  Serial.println("----------------------");
  Serial.println();
  
  // Verbindungsstatus verwalten
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Advertising neu gestartet");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(UPDATE_INTERVAL);
}
