#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP342x.h>

class FuelSensorClass {
public:
    FuelSensorClass(uint8_t address = 0x68);
    
    // Initialization
    bool begin();
    
    // Get fuel level (0-100%)
    float getFuelLevel();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onFuelLevelChange(void (*callback)(float));
    
    // Set fuel level threshold for callback
    void setThreshold(float threshold);

private:
    MCP342x adc;
    
    float currentFuelLevel;
    float lastFuelLevel;
    float fuelThreshold;
    unsigned long lastReadTime;
    
    void (*fuelChangeCallback)(float);
    
    // Read interval in milliseconds
    static const unsigned long READ_INTERVAL = 1000; // Read every 1 second
    
    // Convert ADC reading to fuel level
    float convertToFuelLevel(long adcValue);
};

// Global instance
extern FuelSensorClass FuelSensor; 