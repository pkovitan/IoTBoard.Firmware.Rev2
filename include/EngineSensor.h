#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP23008.h>

class EngineSensorClass {
public:
    EngineSensorClass(uint8_t address = 0x22);
    
    // Initialization
    bool begin();
    
    // Check engine state
    bool isOn();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onEngineStart(void (*callback)());
    void onEngineStop(void (*callback)());

private:
    MCP23008 expander;
    
    bool engineState;       // true = on, false = off
    bool lastEngineState;
    unsigned long lastDebounceTime;
    
    void (*engineStartCallback)();
    void (*engineStopCallback)();
    
    // Pin definition from config
    static const uint8_t ENGINE_PIN = ENGINE_SEN_PIN;
    
    // Debounce time in milliseconds
    static const unsigned long DEBOUNCE_DELAY = 50;
};

// Global instance
extern EngineSensorClass EngineSensor; 