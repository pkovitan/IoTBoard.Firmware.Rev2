#include "LEDIndicator.h"

// Create global instance
LEDIndicator LedIndicator;

LEDIndicator::LEDIndicator(uint8_t address) : expander(address) {
    // Initialize LED states
    redLed = {false, false, false, 0};
    orangeLed = {false, false, false, 0};
    blueLed = {false, false, false, 0};
    cycleMode = false;
    currentColor = 0;
    lastColorChange = 0;
}

bool LEDIndicator::begin() {
    // Set pinMode to OUTPUT for all LEDs
    expander.pinMode(RED_LED_PIN, OUTPUT);
    expander.pinMode(ORANGE_LED_PIN, OUTPUT);
    expander.pinMode(BLUE_LED_PIN, OUTPUT);
    
    // Initialize PCF8574
    if (!expander.begin()) {
        Serial.println("LED Indicator initialization failed!");
        return false;
    }
    
    // Turn off all LEDs initially (HIGH = OFF for common anode LEDs)
    expander.digitalWrite(RED_LED_PIN, HIGH);
    expander.digitalWrite(ORANGE_LED_PIN, HIGH);
    expander.digitalWrite(BLUE_LED_PIN, HIGH);
    
    Serial.println("LED Indicator initialized successfully");
    return true;
}

void LEDIndicator::setRed(bool on, bool blink) {
    redLed.on = on;
    redLed.blink = blink;
}

void LEDIndicator::setOrange(bool on, bool blink) {
    orangeLed.on = on;
    orangeLed.blink = blink;
}

void LEDIndicator::setBlue(bool on, bool blink) {
    blueLed.on = on;
    blueLed.blink = blink;
}

void LEDIndicator::updateLED(LEDState& led, uint8_t pin) {
    if (led.on) {
        if (led.blink) {
            unsigned long currentTime = millis();
            if (currentTime - led.lastToggle >= BLINK_INTERVAL) {
                led.currentState = !led.currentState;
                led.lastToggle = currentTime;
                expander.digitalWrite(pin, !led.currentState); // Inverted because common anode
            }
        } else {
            expander.digitalWrite(pin, LOW); // ON
        }
    } else {
        expander.digitalWrite(pin, HIGH); // OFF
    }
}

void LEDIndicator::startCycleMode() {
    // Reset all LEDs
    setRed(false);
    setOrange(false);
    setBlue(false);
    
    cycleMode = true;
    currentColor = 0;
    lastColorChange = millis();
}

void LEDIndicator::stopCycleMode() {
    cycleMode = false;
    // Turn off all LEDs
    setRed(false);
    setOrange(false);
    setBlue(false);
}

void LEDIndicator::updateCycleMode() {
    if (!cycleMode) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastColorChange >= BLINK_INTERVAL) {
        // Turn off all LEDs first
        setRed(false);
        setOrange(false);
        setBlue(false);
        
        // Change to next color
        switch (currentColor) {
            case 0: // Red
                setRed(true);
                break;
            case 1: // Orange
                setOrange(true);
                break;
            case 2: // Blue
                setBlue(true);
                break;
        }
        
        // Move to next color
        currentColor = (currentColor + 1) % 3;
        lastColorChange = currentTime;
    }
}

void LEDIndicator::update() {
    if (cycleMode) {
        updateCycleMode();
    }
    updateLED(redLed, RED_LED_PIN);
    updateLED(orangeLed, ORANGE_LED_PIN);
    updateLED(blueLed, BLUE_LED_PIN);
} 