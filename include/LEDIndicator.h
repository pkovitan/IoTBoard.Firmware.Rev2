#pragma once

#include <Arduino.h>
#include "config.h"  // Add this to get pin definitions
#include <PCF8574.h>

class LEDIndicator {
public:
    LEDIndicator(uint8_t address = 0x20);
    
    // Initialization
    bool begin();
    
    // LED Control methods
    void setRed(bool on, bool blink = false);
    void setOrange(bool on, bool blink = false);
    void setBlue(bool on, bool blink = false);
    
    // Sequence control
    void startCycleMode();  // Start cycling through colors
    void stopCycleMode();   // Stop cycling
    bool isCycling() const { return cycleMode; }
    
    // Main update loop - should be called in main loop
    void update();

private:
    PCF8574 expander;  // I2C I/O expander
    
    // LED states
    struct LEDState {
        bool on;
        bool blink;
        bool currentState;
        unsigned long lastToggle;
    };
    
    LEDState redLed;
    LEDState orangeLed;
    LEDState blueLed;
    
    // Cycle mode properties
    bool cycleMode;
    uint8_t currentColor;  // 0=Red, 1=Orange, 2=Blue
    unsigned long lastColorChange;
    
    // Pin definitions from original code
    static const uint8_t RED_LED_PIN = PCF8574_RED_PIN;      // PCF8574 0x20 pin P4
    static const uint8_t ORANGE_LED_PIN = PCF8574_ORANGE_PIN;   // PCF8574 0x20 pin P6
    static const uint8_t BLUE_LED_PIN = PCF8574_BLUE_PIN;     // PCF8574 0x20 pin P5
    
    static const unsigned long BLINK_INTERVAL = PCF8574_LED_BLINK_INTERVAL;
    
    // Helper method to update individual LED
    void updateLED(LEDState& led, uint8_t pin);
    void updateCycleMode();
};

// Global instance
extern LEDIndicator LedIndicator; 