#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP23008.h>

class DoorSensorClass {
public:
    DoorSensorClass(uint8_t address = 0x22);
    
    // Initialization
    bool begin();
    
    // Check door state
    bool isOpen();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onOpen(void (*callback)());
    void onClose(void (*callback)());
    
    // Default handlers that can be used in main
    void defaultOpenHandler();
    void defaultCloseHandler();

private:
    MCP23008 expander;
    
    bool doorState;         // true = open, false = closed
    bool lastDoorState;
    unsigned long lastDebounceTime;
    
    void (*openCallback)();
    void (*closeCallback)();
    
    // Pin definition from config
    static const uint8_t DOOR_PIN = DOOR_SW_PIN;
    
    // Debounce time in milliseconds
    static const unsigned long DEBOUNCE_DELAY = 50;
};

// Global instance
extern DoorSensorClass DoorSensor; 