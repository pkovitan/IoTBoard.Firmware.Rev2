#pragma once

#include <Arduino.h>
#include "config.h"

class CardReaderClass {
public:
    CardReaderClass();
    
    // Initialization
    bool begin();
    
    // Get card ID
    const char* getCardID();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onCardDetected(void (*callback)(const char*));
    
private:
    // UART pins for card reader are defined in config.h
    // RXD1 and TXD1 are used directly from there
    
    char cardID[9]; // 8 characters + null terminator
    bool cardDetected;
    unsigned long lastReadTime;
    
    void (*cardDetectedCallback)(const char*);
    
    // Read interval in milliseconds
    static const unsigned long READ_INTERVAL = 1000; // Read every 1 second
    
    // Parse card data from serial input
    void parseCardData();
};

// Global instance
extern CardReaderClass CardReader; 