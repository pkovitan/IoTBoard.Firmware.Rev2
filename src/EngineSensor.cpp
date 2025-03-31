#include "EngineSensor.h"
#include "LEDIndicator.h"

// Create global instance
EngineSensorClass EngineSensor;

EngineSensorClass::EngineSensorClass(uint8_t address) : expander(address) {
    engineState = false;
    lastEngineState = false;
    lastDebounceTime = 0;
    engineStartCallback = nullptr;
    engineStopCallback = nullptr;
}

bool EngineSensorClass::begin() {
    // Initialize MCP23008
    Wire.begin(21, 22); // SDA, SCL pins
    expander.begin(); // Default pullup = true
    expander.pinMode(ENGINE_PIN, INPUT);
    
    // Read initial state (LOW = Engine ON, HIGH = Engine OFF)
    engineState = (expander.digitalRead(ENGINE_PIN) == LOW);
    lastEngineState = engineState;
    
    Serial.println("Engine Sensor initialized successfully");
    return true;
}

bool EngineSensorClass::isOn() {
    return engineState;
}

void EngineSensorClass::onEngineStart(void (*callback)()) {
    engineStartCallback = callback;
}

void EngineSensorClass::onEngineStop(void (*callback)()) {
    engineStopCallback = callback;
}

void EngineSensorClass::update() {
    // Read the current engine state (LOW = Engine ON, HIGH = Engine OFF)
    bool reading = (expander.digitalRead(ENGINE_PIN) == LOW);
    
    // Debug output - show current reading
    static bool lastDebugState = !reading; // Initialize to opposite to force first print
    if (reading != lastDebugState) {
        if (reading) {
            Serial.println("Engine ON");
        } else {
            Serial.println("Engine OFF");
        }
        lastDebugState = reading;
    }
    
    // Check if the engine state has changed
    if (reading != lastEngineState) {
        // Reset the debounce timer
        lastDebounceTime = millis();
    }
    
    // Check if the engine state has been stable for the debounce period
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // If the engine state has changed
        if (reading != engineState) {
            engineState = reading;
            
            // Print status to Serial
            if (engineState) {
                Serial.println("Engine Started");
            } else {
                Serial.println("Engine Stopped");
            }
            
            // Call the appropriate callback function if it exists
            if (engineState && engineStartCallback != nullptr) {
                engineStartCallback(); // Engine started
            } else if (!engineState && engineStopCallback != nullptr) {
                engineStopCallback(); // Engine stopped
            }
        }
    }
    
    // Save the current engine state for the next comparison
    lastEngineState = reading;
}

void EngineSensorClass::defaultStartHandler() {
    Serial.println("Engine Started!");
    // You can add additional actions here when engine starts
    LedIndicator.setBlue(true); // Turn on blue LED when engine starts
}

void EngineSensorClass::defaultStopHandler() {
    Serial.println("Engine Stopped!");
    // You can add additional actions here when engine stops
    LedIndicator.setBlue(false); // Turn off blue LED when engine stops
} 