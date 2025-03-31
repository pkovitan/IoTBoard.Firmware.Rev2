#include "EmergencyButton.h"
#include "LEDIndicator.h"
#include "Buzzer.h"
#include "SensorManager.h"

// Create global instance
EmergencyButtonClass EmergencyButton;

EmergencyButtonClass::EmergencyButtonClass(uint8_t address) : expander(address) {
    emergencyState = false;
    lastButtonState = false;
    lastDebounceTime = 0;
    pressCallback = nullptr;
}

bool EmergencyButtonClass::begin() {
    // Initialize MCP23008
    Wire.begin(21, 22); // SDA, SCL pins
    expander.begin(); // Default pullup = true
    expander.pinMode(EMERGENCY_PIN, INPUT);
    
    Serial.println("Emergency Button initialized successfully");
    return true;
}

bool EmergencyButtonClass::isPressed() {
    return emergencyState;
}

void EmergencyButtonClass::reset() {
    emergencyState = false;
}

void EmergencyButtonClass::onPress(void (*callback)()) {
    pressCallback = callback;
}

void EmergencyButtonClass::update() {
    // Read the current button state
    bool reading = (expander.digitalRead(EMERGENCY_PIN) == LOW);
    
    // Debug output for raw pin reading
    static bool lastReadingDebug = !reading;
    if (reading != lastReadingDebug) {
        Serial.print("Emergency Button Raw Reading: ");
        Serial.println(reading ? "PRESSED" : "RELEASED");
        lastReadingDebug = reading;
    }
    
    // Print current emergency state periodically
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 5000) { // Every 5 seconds
        lastStatusPrint = millis();
        Serial.print("Current Emergency State: ");
        Serial.println(emergencyState ? "TRUE" : "FALSE");
    }
    
    // Check if the button state has changed
    if (reading != lastButtonState) {
        // Reset the debounce timer
        lastDebounceTime = millis();
        
        Serial.println("Button state changed, waiting for debounce...");
    }
    
    // Check if the button state has been stable for the debounce period
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // If the button state has changed
        if (reading != emergencyState) {
            // Update the emergency state
            emergencyState = reading;
            
            if (emergencyState) {
                // Button is now pressed - call the callback
                Serial.println("Emergency Button Pressed - State changed to TRUE");
                if (pressCallback != nullptr) {
                    pressCallback();
                }
            } else {
                // Button is now released
                Serial.println("Emergency Button Released - State changed to FALSE");
                
                // Optionally turn off the red LED when released
                LedIndicator.setRed(false);
                
                // Reset emergency event in SensorManager
                SensorManager.setEvent("NONE");
            }
        }
    }
    
    // Save the current button state for the next comparison
    lastButtonState = reading;
}

void EmergencyButtonClass::defaultPressHandler() {
    Serial.println("Emergency Button Pressed!");
    
    // You can add default emergency actions here
    LedIndicator.setRed(true);
    Buzzer.beep(2000); // Long alarm beep
    
    // Send emergency event to SensorManager
    SensorManager.setEvent("EMERGENCY");
} 