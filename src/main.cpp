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
#include "GPSModule.h"
#include "SensorManager.h"
#include "SDLogger.h"
#include "BuzzerController.h"

// Forward declarations for callback functions that link to handlers in each module
void onEmergencyButtonPress();
void onDoorOpen();
void onDoorClose();
void onEngineStart();
void onEngineStop();
void onTemperatureChange(float temperature);
void onFuelLevelChange(float fuelLevel);
void onCardDetected(const char* cardID);
void onMotionDetected(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
void onVehicleEvent(VehicleEventType eventType, float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
void onLocationChange(double lat, double lon);

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
  
  // Initialize BuzzerController
  if (!BuzzerController.begin()) {
    Serial.println("Failed to initialize BuzzerController!");
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
  
  // Set event reset interval to 10 seconds (events will be cleared from payload after 10 seconds)
  SensorManager.setEventResetInterval(10000);
  
  // Initialize SD Card Logger
  if (!SDLogger.begin(5)) { // SD card CS pin is 5
    Serial.println("Failed to initialize SD Card Logger!");
  }
  
  // Set callbacks for all sensors
  EmergencyButton.onPress(onEmergencyButtonPress);
  DoorSensor.onOpen(onDoorOpen);
  DoorSensor.onClose(onDoorClose);
  EngineSensor.onEngineStart(onEngineStart);
  EngineSensor.onEngineStop(onEngineStop);
  TemperatureSensor.onTemperatureChange(onTemperatureChange);
  TemperatureSensor.setThreshold(2.0);
  FuelSensor.onFuelLevelChange(onFuelLevelChange);
  FuelSensor.setThreshold(5.0);
  CardReader.onCardDetected(onCardDetected);
  AccelerometerGyro.onMotionDetected(onMotionDetected);
  AccelerometerGyro.onVehicleEvent(onVehicleEvent);
  AccelerometerGyro.setThreshold(0.5);
  
  // Set vehicle event thresholds with tuned values for realistic detection
  AccelerometerGyro.setRolloverThreshold(0.8);     
  AccelerometerGyro.setHardBrakeThreshold(0.45);   // Increased from 0.3 to 0.45 to make it harder to trigger
  AccelerometerGyro.setLaneChangeThreshold(0.35);  // Increased from 0.25 to 0.35 to make it harder to trigger
  AccelerometerGyro.setRapidAccelThreshold(0.35);  // Increased from 0.25 to 0.35 to make it less sensitive
  AccelerometerGyro.setSpinThreshold(45.0);        
  AccelerometerGyro.setVibrationThreshold(0.4);    
  
  GPSModule.onLocationChange(onLocationChange);
  GPSModule.setThreshold(0.0001);
  
  // Test double beep at startup to indicate system is ready
  Buzzer.doubleBeep();
  
  // Print system information
  Serial.println("Sensor reading frequency: 50Hz (50 times per second)");
  Serial.println("Data processing rate: ~10Hz (10 times per second)");
  Serial.println("Payload output rate: 1Hz (once per second)");
  Serial.println("System ready!");
  
  // Start periodic beeping to indicate system is ready for card
  BuzzerController.startPeriodicBeep();
}

// Callback handlers that link to module defaults
void onEmergencyButtonPress() {
  EmergencyButton.defaultPressHandler();
}

void onDoorOpen() {
  DoorSensor.defaultOpenHandler();
}

void onDoorClose() {
  DoorSensor.defaultCloseHandler();
}

void onEngineStart() {
  EngineSensor.defaultStartHandler();
}

void onEngineStop() {
  EngineSensor.defaultStopHandler();
}

void onTemperatureChange(float temperature) {
  TemperatureSensor.defaultTemperatureHandler(temperature);
}

void onFuelLevelChange(float fuelLevel) {
  FuelSensor.defaultFuelLevelHandler(fuelLevel);
}

void onCardDetected(const char* cardID) {
  // Call default handler from CardReader
  CardReader.defaultCardHandler(cardID);
  
  // Notify the buzzer controller that a card was detected
  BuzzerController.onCardDetected();
}

void onMotionDetected(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  AccelerometerGyro.defaultMotionHandler(accX, accY, accZ, gyroX, gyroY, gyroZ);
}

void onVehicleEvent(VehicleEventType eventType, float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  AccelerometerGyro.processVehicleEvent(eventType, accX, accY, accZ, gyroX, gyroY, gyroZ);
}

void onLocationChange(double lat, double lon) {
  GPSModule.defaultLocationHandler(lat, lon);
}

void loop() {
    // Variables to track sensor reading rate
    static unsigned long lastRateCheck = 0;
    static int readCounter = 0;
    
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
    BuzzerController.update(); // Add buzzer controller update
    
    // Count each loop iteration for debugging the actual reading rate
    readCounter++;
    
    // Once per 5 seconds, print the reading rate
    if (millis() - lastRateCheck >= 5000) {
        float rate = readCounter / 5.0; // Readings per second
        Serial.print("Sensor reading rate: ");
        Serial.print(rate);
        Serial.println(" Hz");
        
        // Reset counter and timer
        readCounter = 0;
        lastRateCheck = millis();
    }
    
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
    }
    
    // Smaller delay to allow for more frequent sensor readings
    // Was 100ms, reduced to 10ms to enable ~10x faster readings
    delay(10);
} 