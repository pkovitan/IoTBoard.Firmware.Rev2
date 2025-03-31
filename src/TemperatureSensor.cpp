#include "TemperatureSensor.h"
#include "LEDIndicator.h"
#include "Buzzer.h"

// Create global instance
TemperatureSensorClass TemperatureSensor;

TemperatureSensorClass::TemperatureSensorClass(uint8_t address) : adc(address) {
    currentTemperature = 0.0;
    lastTemperature = 0.0;
    temperatureThreshold = 1.0; // Default threshold is 1 degree C
    lastReadTime = 0;
    temperatureChangeCallback = nullptr;
}

bool TemperatureSensorClass::begin() {
    // Reset devices
    MCP342x::generalCallReset();
    delay(1); // MCP342x needs 300us to settle, wait 1ms
    
    // Check device present
    Wire.requestFrom(adc.getAddress(), (uint8_t)1);
    if (!Wire.available()) {
        Serial.print("No ADC device found at address ");
        Serial.println(adc.getAddress(), HEX);
        return false;
    }
    
    Serial.println("Temperature Sensor initialized successfully");
    return true;
}

float TemperatureSensorClass::getTemperature() {
    return currentTemperature;
}

void TemperatureSensorClass::setThreshold(float threshold) {
    temperatureThreshold = threshold;
}

void TemperatureSensorClass::onTemperatureChange(void (*callback)(float)) {
    temperatureChangeCallback = callback;
}

float TemperatureSensorClass::convertToTemperature(long adcValue) {
    // Direct ADC to temperature conversion
    // This approach uses ADC values directly instead of converting to voltage first
    // This provides better precision and reduces conversion errors
    
    // We need to collect ADC values for known temperatures:
    // For now, we'll use placeholder values that you should replace with actual measurements
    
    // Calibration points (replace these with your actual measurements)
    // Format: ADC value, Temperature in °C
    long adc1 = 28059;  // Measured ADC value for cold point (28°C)
    long adc2 = 28939;  // Measured ADC value for hot point (53°C)
    
    float t1 = 28.0;   // Cold point temperature
    float t2 = 53.0;   // Hot point temperature
    
    // Linear interpolation between the closest points
    float temperature;
    
    if (adcValue <= adc1) {
        // Below or at the cold point
        // Linear extrapolation for values below the cold point
        // Using the same slope as between the two known points
        float slope = (t2 - t1) / (float)(adc2 - adc1);
        temperature = t1 - slope * (adc1 - adcValue);
    } else if (adcValue >= adc2) {
        // Above or at the hot point
        // Linear extrapolation for values above the hot point
        // Using the same slope as between the two known points
        float slope = (t2 - t1) / (float)(adc2 - adc1);
        temperature = t2 + slope * (adcValue - adc2);
    } else {
        // Between cold and hot points - linear interpolation
        temperature = t1 + (t2 - t1) * (adcValue - adc1) / (float)(adc2 - adc1);
    }
    
    // Debug calibration values
    static bool printOnce = true;
    if (printOnce) {
        Serial.print("Calibration values - Cold point ADC: ");
        Serial.print(adc1);
        Serial.print(", t1: ");
        Serial.print(t1);
        Serial.print("°C, Hot point ADC: ");
        Serial.print(adc2);
        Serial.print(", t2: ");
        Serial.print(t2);
        Serial.println("°C");
        
        // Calculate and display the slope for reference
        float slope = (t2 - t1) / (float)(adc2 - adc1);
        Serial.print("Temperature slope: ");
        Serial.print(slope);
        Serial.println("°C per ADC unit");
         
        printOnce = false;
    }
    
    // Limit to reasonable temperature range (-50 to 150°C)
    temperature = constrain(temperature, -50.0, 150.0);
    
    // Debug output
    Serial.print("ADC Value: ");
    Serial.print(adcValue);
    Serial.print(", Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
    
    return temperature;
}

void TemperatureSensorClass::update() {
    unsigned long currentTime = millis();
    
    // Read temperature at regular intervals
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Read from ADC
        long adcValue = 0;
        MCP342x::Config status;
        
        // Initiate a conversion and read the result
        uint8_t err = adc.convertAndRead(
            MCP342x::channel4,
            MCP342x::oneShot,
            MCP342x::resolution16,
            MCP342x::gain1,
            1000000, // 1 second timeout
            adcValue,
            status
        );
        
        if (err) {
            Serial.print("Temperature sensor read error: ");
            Serial.println(err);
            return;
        }
        
        // Convert ADC value to temperature
        lastTemperature = currentTemperature;
        currentTemperature = convertToTemperature(adcValue);
        
        // Check if temperature has changed significantly
        if (abs(currentTemperature - lastTemperature) >= temperatureThreshold) {
            // Print temperature change
            Serial.print("Temperature changed: ");
            Serial.print(currentTemperature);
            Serial.println("°C");
            
            // Call callback if registered
            if (temperatureChangeCallback != nullptr) {
                temperatureChangeCallback(currentTemperature);
            }
        }
    }
}

void TemperatureSensorClass::defaultTemperatureHandler(float temperature) {
    Serial.print("Temperature changed to: ");
    Serial.print(temperature);
    Serial.println("°C");
    
    // You can add additional actions here when temperature changes
    // For example, turn on warning if temperature is too high
    if (temperature > 80.0) {
        LedIndicator.setRed(true);
        Buzzer.beep(1000); // Beep for 1 second
    }
} 