#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP23008.h>

class BuzzerClass {
public:
    BuzzerClass(uint8_t address = 0x22);
    
    // Initialization
    bool begin();
    
    // Basic control
    void on();
    void off();
    
    // Pattern controls
    void beep(unsigned int duration);  // Single beep
    void doubleBeep();  // Two short beeps
    void playPattern(const uint8_t* pattern, uint8_t length); // Custom pattern
    
    // Main update loop - should be called in main loop
    void update();

private:
    MCP23008 expander;
    
    // Buzzer state
    bool isOn;
    bool isPlaying;
    unsigned long startTime;
    unsigned long duration;
    
    // Pattern playback
    const uint8_t* currentPattern;
    uint8_t patternLength;
    uint8_t patternIndex;
    
    // Pin definition from config
    static const uint8_t BUZZER_PIN = BUZZER_PIN_MCP;  // Using MCP23008 pin defined in config.h
    
    // Timing constants
    static const unsigned long BEEP_DURATION = 100;  // Default beep duration
    static const unsigned long DOUBLE_BEEP_INTERVAL = 100;  // Interval between double beeps
};

// Global instance
extern BuzzerClass Buzzer; 