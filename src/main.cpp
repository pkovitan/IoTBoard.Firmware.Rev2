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
#include "PayloadModule.h" // Add Payload Module header

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
  
  // Initialize Payload Module
  if (!PayloadModule.begin()) {
    Serial.println("Failed to initialize Payload Module!");
  }
  
  // Set auto print interval to 1 second
  PayloadModule.setPrintInterval(1000);
  
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
  
  // Update Fuel Sensor state
  FuelSensor.update();
  
  // Update Card Reader state
  CardReader.update();
  
  // Update Accelerometer & Gyro state
  AccelerometerGyro.update();
  
  // Update GPS Module state
  GPSModule.update();
  
  // Update Payload Module with current sensor values
  updatePayloadData();
  
  // Update Payload Module state
  PayloadModule.update();
  
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
  
  // You can also check fuel level directly if needed
  float currentFuel = FuelSensor.getFuelLevel();
  if (currentFuel < 5.0) {
    // Fuel level is critically low
    // Take emergency action
  }
  
  // You can also check card ID directly if needed
  const char* currentCardID = CardReader.getCardID();
  if (strcmp(currentCardID, "00000000") != 0) {
    // A card has been detected
    // Take appropriate action
  }
  
  // You can also check GPS data directly if needed
  if (GPSModule.hasValidFix()) {
    double lat = GPSModule.getLatitude();
    double lon = GPSModule.getLongitude();
    float speed = GPSModule.getSpeed();
    
    // Check if vehicle is speeding
    if (speed > 120.0) {
      // Vehicle is speeding
      // Take appropriate action
      LedIndicator.setRed(true);
      Buzzer.beep(500); // Warning beep
    }
  }
  
  // You can also check accelerometer values directly if needed
  float currentAccX = AccelerometerGyro.getAccX();
  float currentAccY = AccelerometerGyro.getAccY();
  float currentAccZ = AccelerometerGyro.getAccZ();
  
  // Check for vehicle tilt
  if (abs(currentAccY) > 0.5) {
    // Vehicle is tilted sideways
    // Take appropriate action
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

// Function to update payload data from all sensors
void updatePayloadData() {
  // Set ID and fleet (could be loaded from configuration)
  PayloadModule.setID("Vehicle01");
  PayloadModule.setFleet("WillDeleteLater");
  
  // Set event (could be determined by system state)
  PayloadModule.setEvent("WillDeleteLater");
  
  // Set card reader data
  PayloadModule.setCardReader(CardReader.getCardID());
  
  // Set voltage data
  PayloadModule.setVoltageIn(VoltageMonitor.getVinVoltage());
  PayloadModule.setVoltageBattery(VoltageMonitor.getBatteryVoltage());
  
  // Set engine state
  PayloadModule.setEngine(EngineSensor.isOn() ? "ON" : "OFF");
  
  // Set GPS data
  if (GPSModule.hasValidFix()) {
    PayloadModule.setSpeed(GPSModule.getSpeed());
    PayloadModule.setDirection(GPSModule.getDirection());
    PayloadModule.setLocation(GPSModule.getLatitude(), GPSModule.getLongitude());
  }
  
  // Set fuel level
  PayloadModule.setFuel(FuelSensor.getFuelLevel());
  
  // Set door state
  PayloadModule.setDoor(DoorSensor.isOpen() ? "OPEN" : "CLOSE");
  
  // Set temperature
  PayloadModule.setTemperature(TemperatureSensor.getTemperature());
  
  // Set accelerometer data
  PayloadModule.setAccelerometer(
    AccelerometerGyro.getAccX(),
    AccelerometerGyro.getAccY(),
    AccelerometerGyro.getAccZ()
  );
  
  // Set gyroscope data
  PayloadModule.setGyroscope(
    AccelerometerGyro.getGyroX(),
    AccelerometerGyro.getGyroY(),
    AccelerometerGyro.getGyroZ()
  );
  
  // Set emergency state
  PayloadModule.setEmergency(EmergencyButton.isPressed() ? "TRUE" : "FALSE");
} 