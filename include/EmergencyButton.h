#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP23008.h>

class EmergencyButtonClass {
public:
    EmergencyButtonClass(uint8_t address = 0x22);
    
    // Initialization
    bool begin();
    
    // Check button state
    bool isPressed();
    
    // Reset emergency state
    void reset();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callback
    void onPress(void (*callback)());

private:
    MCP23008 expander;
    
    bool emergencyState;
    bool lastButtonState;
    unsigned long lastDebounceTime;
    
    void (*pressCallback)();
    
    // Pin definition from config
    static const uint8_t EMERGENCY_PIN = EMERGENCY_SW_PIN;
    
    // Debounce time in milliseconds
    static const unsigned long DEBOUNCE_DELAY = 50;
};

// Global instance
extern EmergencyButtonClass EmergencyButton; 