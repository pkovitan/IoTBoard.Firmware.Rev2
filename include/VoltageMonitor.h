#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP342x.h>

class VoltageMonitorClass {
public:
    VoltageMonitorClass(uint8_t address = 0x68);
    
    // Initialization
    bool begin();
    
    // Get voltage values
    float getVin();
    float getVbatt();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Get voltage values (alternative names)
    float getVinVoltage();
    float getBatteryVoltage();
    
    // Event callbacks
    void onVoltageChange(void (*callback)(float, float));
    
private:
    MCP342x adc;
    
    float _vinVoltage;
    float _vbattVoltage;
    unsigned long lastReadTime;
    
    // Read interval in milliseconds
    static const unsigned long READ_INTERVAL = 1000; // Read every 1 second
};

// Global instance
extern VoltageMonitorClass VoltageMonitor; 