#include "BuzzerController.h"

// Create global instance
BuzzerControllerClass BuzzerController;

BuzzerControllerClass::BuzzerControllerClass() {
    periodicBeepEnabled = false;
    lastBeepTime = 0;
    cardDetected = false;
}

bool BuzzerControllerClass::begin() {
    // No specific initialization needed as we're using the Buzzer class
    periodicBeepEnabled = false;
    lastBeepTime = 0;
    cardDetected = false;
    
    Serial.println("BuzzerController initialized successfully");
    return true;
}

void BuzzerControllerClass::startPeriodicBeep() {
    periodicBeepEnabled = true;
    lastBeepTime = millis() - BEEP_INTERVAL; // Start the first beep immediately
    Serial.println("Periodic beep started");
}

void BuzzerControllerClass::stopPeriodicBeep() {
    periodicBeepEnabled = false;
    Serial.println("Periodic beep stopped");
}

bool BuzzerControllerClass::isPeriodicBeepActive() {
    return periodicBeepEnabled;
}

void BuzzerControllerClass::onCardDetected() {
    // When a card is detected, we'll beep once and then stop periodic beeping
    Buzzer.beep(BEEP_DURATION);
    
    // Stop the periodic beep
    stopPeriodicBeep();
    
    // Set the card detected flag
    cardDetected = true;
    
    Serial.println("Card detected, periodic beep disabled");
}

void BuzzerControllerClass::update() {
    if (!periodicBeepEnabled) {
        return; // Do nothing if periodic beep is disabled
    }
    
    unsigned long currentTime = millis();
    
    // Beep every 5 seconds if enabled
    if (currentTime - lastBeepTime >= BEEP_INTERVAL) {
        lastBeepTime = currentTime;
        
        // Perform a single short beep
        Buzzer.beep(BEEP_DURATION);
        
        Serial.println("Periodic beep");
    }
} 