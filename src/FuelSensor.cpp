#include "FuelSensor.h"
#include "LEDIndicator.h"
#include "Buzzer.h"

// Create global instance
FuelSensorClass FuelSensor;

FuelSensorClass::FuelSensorClass(uint8_t address) : adc(address) {
    currentFuelLevel = 0.0;
    lastFuelLevel = 0.0;
    fuelThreshold = 1.0; // Default threshold is 1%
    lastReadTime = 0;
    fuelChangeCallback = nullptr;
}

bool FuelSensorClass::begin() {
    // ADC should already be initialized by TemperatureSensor
    // Just print a message
    Serial.println("Fuel Sensor initialized successfully");
    return true;
}

float FuelSensorClass::getFuelLevel() {
    return currentFuelLevel;
}

void FuelSensorClass::setThreshold(float threshold) {
    fuelThreshold = threshold;
}

void FuelSensorClass::onFuelLevelChange(void (*callback)(float)) {
    fuelChangeCallback = callback;
}

float FuelSensorClass::convertToFuelLevel(long adcValue) {
    // Convert ADC value to voltage
    float vref = 2.048;
    float voltage = (adcValue / 32767.0) * vref;
    
    // Convert ADC value to fuel level (0-100%)
    // Calibration data:
    // - Full tank (100%): ADC Value = 1170
    // - Empty tank (0%): ADC Value = 0
    
    // Linear mapping from ADC value to percentage
    float fuelLevel = (adcValue / 1170.0) * 100.0;
    
    // Limit to valid range (0-100%)
    fuelLevel = constrain(fuelLevel, 0.0, 100.0);
    
    // Debug output
    Serial.print("Fuel ADC Value: ");
    Serial.print(adcValue);
    Serial.print(", Voltage: ");
    Serial.print(voltage);
    Serial.print("V, Fuel Level: ");
    Serial.print(fuelLevel);
    Serial.println("%");
    
    return fuelLevel;
}

void FuelSensorClass::update() {
    unsigned long currentTime = millis();
    
    // Read fuel level at regular intervals
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Read from ADC
        long adcValue = 0;
        MCP342x::Config status;
        
        // Initiate a conversion and read the result
        uint8_t err = adc.convertAndRead(
            MCP342x::channel3,  // FUEL_SEN is on channel3
            MCP342x::oneShot,
            MCP342x::resolution16,
            MCP342x::gain1,
            1000000, // 1 second timeout
            adcValue,
            status
        );
        
        if (err) {
            Serial.print("Fuel sensor read error: ");
            Serial.println(err);
            return;
        }
        
        // Convert ADC value to fuel level
        lastFuelLevel = currentFuelLevel;
        currentFuelLevel = convertToFuelLevel(adcValue);
        
        // Check if fuel level has changed significantly
        if (abs(currentFuelLevel - lastFuelLevel) >= fuelThreshold) {
            // Print fuel level change
            Serial.print("Fuel level changed: ");
            Serial.print(currentFuelLevel);
            Serial.println("%");
            
            // Call callback if registered
            if (fuelChangeCallback != nullptr) {
                fuelChangeCallback(currentFuelLevel);
            }
        }
    }
}

void FuelSensorClass::defaultFuelLevelHandler(float fuelLevel) {
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