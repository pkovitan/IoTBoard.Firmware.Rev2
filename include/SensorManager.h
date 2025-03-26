#pragma once

#include <Arduino.h>
#include "config.h"
#include "system.h"
#include "LEDIndicator.h"
#include "Buzzer.h"
#include "CardReader.h"
#include "VoltageMonitor.h"
#include "EngineSensor.h"
#include "GPSModule.h"
#include "FuelSensor.h"
#include "DoorSensor.h"
#include "TemperatureSensor.h"
#include "AccelerometerGyro.h"
#include "EmergencyButton.h"

// Payload formats supported by SensorManager
enum PayloadFormat {
    JSON_FORMAT,
    NMEA_FORMAT
};

class SensorManagerClass {
public:
    SensorManagerClass();
    
    // Initialization
    bool begin();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Read data from all sensors
    void readAllSensors();
    
    // Get payload data in specified format
    const char* getPayload(PayloadFormat format = JSON_FORMAT);
    
    // Print payload to Serial in specified format
    void printPayload(PayloadFormat format = JSON_FORMAT);
    
    // Enable/disable automatic printing
    void enableAutoPrint(bool enable);
    
    // Set print interval
    void setPrintInterval(unsigned long interval);
    
    // Set fleet and ID information
    void setFleet(const char* fleet);
    void setID(const char* id);
    void setEvent(const char* event);

private:
    // Sensor data
    char fleet[32];
    char id[32];
    uint16_t seq;
    char timeString[32];
    char event[32];
    char cardReader[16];
    float vin;
    float vbat;
    char engine[8];
    float speed;
    float direction;
    float fuel;
    double latitude;
    double longitude;
    char door[8];
    float temp;
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    char emergency[8];
    
    // Payload buffers
    char jsonPayload[1024];
    char nmeaPayload[1024];
    
    // Auto print settings
    bool autoPrintEnabled;
    unsigned long printInterval;
    unsigned long lastPrintTime;
    PayloadFormat autoPrintFormat;
    
    // Update payload strings
    void updateJsonPayload();
    void updateNmeaPayload();
    
    // Update time string
    void updateTimeString();
    
    // NMEA checksum calculation
    uint8_t calculateNmeaChecksum(const char* sentence);
};

// Global instance
extern SensorManagerClass SensorManager; 