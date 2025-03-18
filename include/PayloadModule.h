#pragma once

#include <Arduino.h>
#include "config.h"

class PayloadModuleClass {
public:
    PayloadModuleClass();
    
    // Initialization
    bool begin();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Set payload data
    void setFleet(const char* fleet);
    void setID(const char* id);
    void setEvent(const char* event);
    void setCardReader(const char* cardID);
    void setVoltageIn(float vin);
    void setVoltageBattery(float vbat);
    void setEngine(const char* engine);
    void setSpeed(float speed);
    void setDirection(float direction);
    void setFuel(float fuel);
    void setLocation(double latitude, double longitude);
    void setDoor(const char* door);
    void setTemperature(float temp);
    void setAccelerometer(float x, float y, float z);
    void setGyroscope(float x, float y, float z);
    void setEmergency(const char* emergency);
    
    // Get payload as JSON string
    const char* getPayload();
    
    // Print payload to Serial
    void printPayload();
    
    // Enable/disable automatic printing
    void enableAutoPrint(bool enable);
    
    // Set print interval
    void setPrintInterval(unsigned long interval);

private:
    // Payload data
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
    
    // Payload buffer
    char payload[1024];
    
    // Auto print settings
    bool autoPrintEnabled;
    unsigned long printInterval;
    unsigned long lastPrintTime;
    
    // Update payload string
    void updatePayload();
    
    // Update time string
    void updateTimeString();
};

// Global instance
extern PayloadModuleClass PayloadModule; 