/**
 * INTrackG v1.0
 * Rewritten by [Your Name]
 * Based on original work by WATT
 * 
 * A vehicle tracking and monitoring system
 */

#include <Arduino.h>
#include "config.h"
#include "system.h"
#include "LEDIndicator.h"
#include "Buzzer.h"
#include "EmergencyButton.h"
#include "DoorSensor.h"
#include "EngineSensor.h"
#include "TemperatureSensor.h"
#include "VoltageMonitor.h"

// Callback function for emergency button press
void emergencyButtonPressed() {
  Serial.println("Emergency Button Pressed!");
}

// Callback functions for door sensor
void doorOpened() {
  Serial.println("Door Opened!");
}

void doorClosed() {
  Serial.println("Door Closed!");
}

// Callback functions for engine sensor
void engineStarted() {
  Serial.println("Engine Started!");
  // You can add additional actions here when engine starts
  LedIndicator.setBlue(true); // Turn on blue LED when engine starts
}

void engineStopped() {
  Serial.println("Engine Stopped!");
  // You can add additional actions here when engine stops
  LedIndicator.setBlue(false); // Turn off blue LED when engine stops
}

// Callback function for temperature change
void temperatureChanged(float temperature) {
  Serial.print("Temperature changed to: ");
  Serial.print(temperature);
  Serial.println("°C");
  
  // You can add additional actions here when temperature changes
  // For example, turn on warning if temperature is too high
  if (temperature > 80.0) {
    LedIndicator.setRed(true);
    Buzzer.beep(1000); // Beep for 1 second
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== INTrackG v1.0 ===");
  Serial.println("System starting...");
  
  // Initialize LED Indicator
  if (!LedIndicator.begin()) {
    Serial.println("Failed to initialize LED Indicator!");
  }
  
  // Initialize Buzzer
  if (!Buzzer.begin()) {
    Serial.println("Failed to initialize Buzzer!");
  }
  
  // Initialize Emergency Button
  if (!EmergencyButton.begin()) {
    Serial.println("Failed to initialize Emergency Button!");
  }
  
  // Initialize Door Sensor
  if (!DoorSensor.begin()) {
    Serial.println("Failed to initialize Door Sensor!");
  }
  
  // Initialize Engine Sensor
  if (!EngineSensor.begin()) {
    Serial.println("Failed to initialize Engine Sensor!");
  }
  
  // Initialize Temperature Sensor
  if (!TemperatureSensor.begin()) {
    Serial.println("Failed to initialize Temperature Sensor!");
  }
  
  // Initialize Voltage Monitor
  if (!VoltageMonitor.begin()) {
    Serial.println("Failed to initialize Voltage Monitor!");
  }
  
  // Set callback for emergency button press
  EmergencyButton.onPress(emergencyButtonPressed);
  
  // Set callbacks for door sensor
  DoorSensor.onOpen(doorOpened);
  DoorSensor.onClose(doorClosed);
  
  // Set callbacks for engine sensor
  EngineSensor.onEngineStart(engineStarted);
  EngineSensor.onEngineStop(engineStopped);
  
  // Set callback for temperature sensor
  TemperatureSensor.onTemperatureChange(temperatureChanged);
  
  // Set temperature threshold to 2 degrees (callback will be triggered when temperature changes by 2°C or more)
  TemperatureSensor.setThreshold(2.0);
  
  // Test double beep at startup
  Buzzer.doubleBeep();
  
  // Initialize system components
  System.init();
}

void loop() {
  // Main system loop
  System.update();
  
  // Update LED states
  LedIndicator.update();
  
  // Update Buzzer states
  Buzzer.update();
  
  // Update Emergency Button state
  EmergencyButton.update();
  
  // Update Door Sensor state
  DoorSensor.update();
  
  // Update Engine Sensor state
  EngineSensor.update();
  
  // Update Temperature Sensor state
  TemperatureSensor.update();
  
  // Update Voltage Monitor state
  VoltageMonitor.update();
  
  // Test emergency button state
  if (EmergencyButton.isPressed()) {
    Serial.println("Emergency Button is currently pressed!");
    // Reset the emergency state after handling it
    EmergencyButton.reset();
  }
  
  // You can also check door state directly if needed
  if (DoorSensor.isOpen()) {
    // Door is currently open
  } else {
    // Door is currently closed
  }
  
  // You can also check engine state directly if needed
  if (EngineSensor.isOn()) {
    // Engine is currently running
  } else {
    // Engine is currently off
  }
  
  // You can also check temperature directly if needed
  float currentTemp = TemperatureSensor.getTemperature();
  if (currentTemp > 90.0) {
    // Temperature is critically high
    // Take emergency action
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
} 