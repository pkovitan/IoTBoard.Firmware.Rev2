#include "DoorSensor.h"

// Create global instance
DoorSensorClass DoorSensor;

DoorSensorClass::DoorSensorClass(uint8_t address) : expander(address) {
    doorState = false;
    lastDoorState = false;
    lastDebounceTime = 0;
    openCallback = nullptr;
    closeCallback = nullptr;
}

bool DoorSensorClass::begin() {
    // Initialize MCP23008
    Wire.begin(21, 22); // SDA, SCL pins
    expander.begin(); // Default pullup = true
    expander.pinMode(DOOR_PIN, INPUT);
    
    // Read initial state
    doorState = (expander.digitalRead(DOOR_PIN) == HIGH);
    lastDoorState = doorState;
    
    Serial.println("Door Sensor initialized successfully");
    return true;
}

bool DoorSensorClass::isOpen() {
    return doorState;
}

void DoorSensorClass::onOpen(void (*callback)()) {
    openCallback = callback;
}

void DoorSensorClass::onClose(void (*callback)()) {
    closeCallback = callback;
}

void DoorSensorClass::update() {
    // Read the current door state
    bool reading = (expander.digitalRead(DOOR_PIN) == HIGH);
    
    // Check if the door state has changed
    if (reading != lastDoorState) {
        // Reset the debounce timer
        lastDebounceTime = millis();
    }
    
    // Check if the door state has been stable for the debounce period
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // If the door state has changed
        if (reading != doorState) {
            doorState = reading;
            
            // Print status to Serial
            if (doorState) {
                Serial.println("Door Open");
            } else {
                Serial.println("Door Close");
            }
            
            // Call the appropriate callback function if it exists
            if (doorState && openCallback != nullptr) {
                openCallback(); // Door opened
            } else if (!doorState && closeCallback != nullptr) {
                closeCallback(); // Door closed
            }
        }
    }
    
    // Save the current door state for the next comparison
    lastDoorState = reading;
}

// Add after other methods
void DoorSensorClass::defaultOpenHandler() {
    Serial.println("Door Opened!");
}

void DoorSensorClass::defaultCloseHandler() {
    Serial.println("Door Closed!");
} 