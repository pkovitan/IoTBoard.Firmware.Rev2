#include "VoltageMonitor.h"

// Create global instance
VoltageMonitorClass VoltageMonitor;

VoltageMonitorClass::VoltageMonitorClass(uint8_t address) : adc(address) {
    _vinVoltage = 0.0;
    _vbattVoltage = 0.0;
    lastReadTime = 0;
}

bool VoltageMonitorClass::begin() {
    // ADC should already be initialized by TemperatureSensor
    // Just print a message
    Serial.println("Voltage Monitor initialized successfully");
    return true;
}

float VoltageMonitorClass::getVin() {
    return _vinVoltage;
}

float VoltageMonitorClass::getVbatt() {
    return _vbattVoltage;
}

void VoltageMonitorClass::update() {
    unsigned long currentTime = millis();
    
    // Read voltages at regular intervals
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Read VIN
        long adcValue = 0;
        MCP342x::Config status;
        
        // Read VIN_SEN (channel2)
        uint8_t err = adc.convertAndRead(
            MCP342x::channel2,
            MCP342x::oneShot,
            MCP342x::resolution16,
            MCP342x::gain1,
            1000000, // 1 second timeout
            adcValue,
            status
        );
        
        if (!err) {
            // Convert ADC value to voltage (based on Legacy-Main code)
            float vref = 2.048;
            float rawVoltage = (adcValue / 32767.0) * vref;
            
            // Apply scaling factor from Legacy-Main (7.14)
            _vinVoltage = rawVoltage * 7.14;
            
            // Debug output
            Serial.print("VIN: ");
            Serial.print(_vinVoltage);
            Serial.println("V");
        } else {
            Serial.print("VIN sensor read error: ");
            Serial.println(err);
        }
        
        // Read BATT_SEN (channel1)
        err = adc.convertAndRead(
            MCP342x::channel1,
            MCP342x::oneShot,
            MCP342x::resolution16,
            MCP342x::gain1,
            1000000, // 1 second timeout
            adcValue,
            status
        );
        
        if (!err) {
            // Convert ADC value to voltage (based on Legacy-Main code)
            float vref = 2.048;
            float rawVoltage = (adcValue / 32767.0) * vref;
            
            // Apply scaling factor from Legacy-Main (111.0/75.0)
            _vbattVoltage = rawVoltage * (111.0/75.0);
            
            // Debug output
            Serial.print("VBATT: ");
            Serial.print(_vbattVoltage);
            Serial.println("V");
        } else {
            Serial.print("VBATT sensor read error: ");
            Serial.println(err);
        }
    }
}

float VoltageMonitorClass::getVinVoltage() {
    return _vinVoltage;
}

float VoltageMonitorClass::getBatteryVoltage() {
    return _vbattVoltage;
} 