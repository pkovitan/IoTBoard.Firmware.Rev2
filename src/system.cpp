#include "system.h"
#include "config.h"

SystemClass::SystemClass() : lastUpdateTime(0) {
  // Constructor
}

void SystemClass::init() {
  initSerial();
  initLEDs();
  
  Serial.println("System initialized successfully");
}

void SystemClass::update() {
  unsigned long currentTime = millis();
  
  // Perform periodic tasks
  if (currentTime - lastUpdateTime >= 1000) {
    lastUpdateTime = currentTime;
    
    // 1-second periodic tasks
    if (DEBUG) {
      Serial.print(".");
    }
  }
}

void SystemClass::initSerial() {
  // Serial already initialized in main.cpp
  Serial.println("Serial communication initialized");
}

void SystemClass::initLEDs() {
  // LED initialization will be implemented later
  Serial.println("LED system initialized");
}

// Create global instance
SystemClass System; 