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
#include "FuelSensor.h"
#include "CardReader.h"
#include "AccelerometerGyro.h"
#include "GPSModule.h"  // Add GPS Module header
#include "SensorManager.h" // Add Sensor Manager header
#include "SDLogger.h" // Add SD Card Logger header

// Function prototype
void updatePayloadData();

// Callback function for emergency button press
void emergencyButtonPressed() {
  Serial.println("Emergency Button Pressed!");
}

// Callback function for GPS location change
void locationChanged(double lat, double lon) {
  Serial.print("Location changed to: ");
  Serial.print(lat, 7);
  Serial.print(", ");
  Serial.println(lon, 7);
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

// Callback function for fuel level change
void fuelLevelChanged(float fuelLevel) {
  Serial.print("Fuel level changed to: ");
  Serial.print(fuelLevel);
  Serial.println("%");
  
  // You can add additional actions here when fuel level changes
  // For example, turn on warning if fuel level is too low
  if (fuelLevel < 10.0) {
    LedIndicator.setOrange(true);
    Buzzer.beep(500); // Beep for 0.5 second
  }
}

// Callback function for card detection
void cardDetected(const char* cardID) {
  Serial.print("Card detected: ");
  Serial.println(cardID);
  
  // You can add additional actions here when a card is detected
  // For example, validate the card ID against a list of authorized cards
  LedIndicator.setRed(true);
  Buzzer.beep(200); // Short beep for card detection
  
  // Turn off green LED after 2 seconds
  delay(2000);
  LedIndicator.setRed(false);
}

// Callback function for motion detection
void motionDetected(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  Serial.println("Significant motion detected!");
  Serial.print("Acceleration: X=");
  Serial.print(accX);
  Serial.print("g, Y=");
  Serial.print(accY);
  Serial.print("g, Z=");
  Serial.print(accZ);
  Serial.println("g");
  
  Serial.print("Gyroscope: X=");
  Serial.print(gyroX);
  Serial.print("°/s, Y=");
  Serial.print(gyroY);
  Serial.print("°/s, Z=");
  Serial.print(gyroZ);
  Serial.println("°/s");
  
  // You can add additional actions here when motion is detected
  // For example, alert if there's a sudden impact or tilt
  if (abs(accX) > 1.5 || abs(accY) > 1.5 || abs(accZ) > 1.5) {
    LedIndicator.setRed(true);
    Buzzer.beep(300); // Beep for impact detection
    delay(1000);
    LedIndicator.setRed(false);
  }
}

// Callback function for reading log file
void printLogLine(const char* line) {
    Serial.println(line);
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
  
  // Initialize Fuel Sensor
  if (!FuelSensor.begin()) {
    Serial.println("Failed to initialize Fuel Sensor!");
  }
  
  // Initialize Card Reader
  if (!CardReader.begin()) {
    Serial.println("Failed to initialize Card Reader!");
  }
  
  // Initialize Accelerometer & Gyro
  if (!AccelerometerGyro.begin()) {
    Serial.println("Failed to initialize Accelerometer & Gyro!");
  }
  
  // Initialize GPS Module
  if (!GPSModule.begin()) {
    Serial.println("Failed to initialize GPS Module!");
  }
  
  // Initialize Sensor Manager
  if (!SensorManager.begin()) {
    Serial.println("Failed to initialize Sensor Manager!");
  }
  
  // Set auto print interval to 1 second
  SensorManager.setPrintInterval(1000);
  
  // Initialize SD Card Logger
  if (!SDLogger.begin(5)) { // SD card CS pin is 5
    Serial.println("Failed to initialize SD Card Logger!");
  } else {
    // Format SD card at startup
    Serial.println("Formatting SD card...");
    if (SDLogger.formatSDCard()) {
      Serial.println("SD card formatted successfully");
    } else {
      Serial.println("Failed to format SD card");
    }
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
  
  // Set callback for fuel sensor
  FuelSensor.onFuelLevelChange(fuelLevelChanged);
  
  // Set fuel threshold to 5% (callback will be triggered when fuel level changes by 5% or more)
  FuelSensor.setThreshold(5.0);
  
  // Set callback for card reader
  CardReader.onCardDetected(cardDetected);
  
  // Set callback for accelerometer/gyro
  AccelerometerGyro.onMotionDetected(motionDetected);
  
  // Set motion threshold to 0.5g (callback will be triggered when acceleration changes by 0.5g or more)
  AccelerometerGyro.setThreshold(0.5);
  
  // Set callback for GPS location change
  GPSModule.onLocationChange(locationChanged);
  
  // Set location threshold to 0.0001 degrees (approximately 10 meters)
  GPSModule.setThreshold(0.0001);
  
  // Test double beep at startup
  Buzzer.doubleBeep();
}

void loop() {
    // Update all sensors
    CardReader.update();
    VoltageMonitor.update();
    EngineSensor.update();
    GPSModule.update();
    FuelSensor.update();
    DoorSensor.update();
    TemperatureSensor.update();
    AccelerometerGyro.update();
    EmergencyButton.update();
    
    // Update SensorManager
    SensorManager.update();
    
    // Toggle between JSON and NMEA formats every 5 seconds
    static unsigned long lastFormatToggle = 0;
    static bool useJsonFormat = true;
    if (millis() - lastFormatToggle >= 5000) {
        lastFormatToggle = millis();
        useJsonFormat = !useJsonFormat;
    }
    
    // Get current payload based on selected format
    const char* payload = SensorManager.getPayload(useJsonFormat ? JSON_FORMAT : NMEA_FORMAT);
    
    // Print payload to serial
    Serial.println(payload);
    
    // Log to SD card
    if (SDLogger.isCardMounted()) {
        SDLogger.writeLog(payload);
        
        // Print SD card statistics and log contents every 10 seconds
        static unsigned long lastStatsPrint = 0;
        if (millis() - lastStatsPrint >= 10 * 1000) { // 10 seconds in milliseconds
            lastStatsPrint = millis();
            
            Serial.println("\n=== SD Card Statistics ===");
            Serial.printf("Used: %lluMB / Total: %lluMB\n", 
                SDLogger.getUsedBytes() / (1024 * 1024), 
                SDLogger.getTotalBytes() / (1024 * 1024)
            );
            Serial.println("=====================\n");
            
            // Read and display log file contents
            Serial.println("\n=== Log File Contents ===");
            Serial.printf("Current file: %s\n", SDLogger.getCurrentFileName());
            if (SDLogger.readLog(SDLogger.getCurrentFileName(), printLogLine)) {
                Serial.println("Log file read successfully");
            } else {
                Serial.println("Failed to read log file");
            }
            Serial.println("=====================\n");
        }
    }
    
    // Small delay to prevent overwhelming the serial output
    delay(100);
} 