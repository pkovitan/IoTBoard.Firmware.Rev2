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
  
  // Small delay to prevent CPU hogging
  delay(10);
} 