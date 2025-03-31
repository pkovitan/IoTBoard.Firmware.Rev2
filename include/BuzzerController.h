#pragma once

#include <Arduino.h>
#include "Buzzer.h"

class BuzzerControllerClass {
public:
    BuzzerControllerClass();
    
    // Initialization
    bool begin();
    
    // Start periodic beeping (every 5 seconds)
    void startPeriodicBeep();
    
    // Stop periodic beeping
    void stopPeriodicBeep();
    
    // Check if periodic beeping is active
    bool isPeriodicBeepActive();
    
    // Card detected event
    void onCardDetected();
    
    // Main update loop - should be called in main loop
    void update();
    
private:
    bool periodicBeepEnabled;
    unsigned long lastBeepTime;
    bool cardDetected;
    
    // Beep interval in milliseconds (5 seconds)
    static const unsigned long BEEP_INTERVAL = 5000;
    
    // Beep duration
    static const unsigned int BEEP_DURATION = 100; // 100ms for short beep
};

// Global instance
extern BuzzerControllerClass BuzzerController; 