#include "EmergencyButton.h"

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
    
    // Check if the button state has changed
    if (reading != lastButtonState) {
        // Reset the debounce timer
        lastDebounceTime = millis();
    }
    
    // Check if the button state has been stable for the debounce period
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // If the button state has changed and is now pressed
        if (reading && !emergencyState) {
            emergencyState = true;
            
            // Call the callback function if it exists
            if (pressCallback != nullptr) {
                pressCallback();
            }
        }
    }
    
    // Save the current button state for the next comparison
    lastButtonState = reading;
} 