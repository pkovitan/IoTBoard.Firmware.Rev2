#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP342x.h>
#include "LEDIndicator.h"
#include "Buzzer.h"

class TemperatureSensorClass {
public:
    TemperatureSensorClass(uint8_t address = 0x68);
    
    // Initialization
    bool begin();
    
    // Get temperature value
    float getTemperature();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onTemperatureChange(void (*callback)(float));
    
    // Set temperature threshold for callback
    void setThreshold(float threshold);
    
    // Default handler that can be used in main
    void defaultTemperatureHandler(float temperature);

private:
    MCP342x adc;
    
    float currentTemperature;
    float lastTemperature;
    float temperatureThreshold;
    unsigned long lastReadTime;
    
    void (*temperatureChangeCallback)(float);
    
    // ADC channel for PT100
    // Use channel 4 for PT100 (defined in update method)
    
    // Read interval in milliseconds
    static const unsigned long READ_INTERVAL = 1000;
    
    // Convert ADC reading to temperature
    float convertToTemperature(long adcValue);
};

// Global instance
extern TemperatureSensorClass TemperatureSensor; 