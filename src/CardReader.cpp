#include "CardReader.h"

// Create global instance
CardReaderClass CardReader;

CardReaderClass::CardReaderClass() {
    strcpy(cardID, "00000000"); // Default empty card ID
    cardDetected = false;
    lastReadTime = 0;
    cardDetectedCallback = nullptr;
}

bool CardReaderClass::begin() {
    // Initialize UART1 for card reader
    Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
    Serial.println("Card Reader initialized successfully");
    return true;
}

const char* CardReaderClass::getCardID() {
    return cardID;
}

void CardReaderClass::onCardDetected(void (*callback)(const char*)) {
    cardDetectedCallback = callback;
}

void CardReaderClass::parseCardData() {
    if (Serial1.available()) {
        // Buffer to store incoming data
        char buffer[256];
        int bytesRead = Serial1.readBytes(buffer, sizeof(buffer) - 1);
        buffer[bytesRead] = '\0'; // Null-terminate the string
        
        Serial.print("Card Reader Raw Data: ");
        Serial.println(buffer);
        
        // Look for the card ID pattern based on Legacy-Main code
        char* index = strchr(buffer, '+');
        if (index != NULL) {
            // Found the '+' character, now extract the card ID
            // Based on Legacy-Main, the card ID is 43 characters after '+'
            if (strlen(index) > 43) {
                bool idFound = false;
                for (int i = 0; i < 8; i++) {
                    if (*(index + 43 + i) != ' ') {
                        cardID[i] = *(index + 43 + i);
                        idFound = true;
                    } else {
                        cardID[i] = '0'; // Fill with '0' if space is found
                    }
                }
                cardID[8] = '\0'; // Ensure null-termination
                
                if (idFound) {
                    cardDetected = true;
                    Serial.print("Card ID detected: ");
                    Serial.println(cardID);
                    
                    // Call callback if registered
                    if (cardDetectedCallback != nullptr) {
                        cardDetectedCallback(cardID);
                    }
                }
            }
        }
    }
}

void CardReaderClass::update() {
    unsigned long currentTime = millis();
    
    // Read card reader at regular intervals
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Parse card data from serial input
        parseCardData();
        
        // Print card ID periodically
        Serial.print("Current Card ID: ");
        Serial.println(cardID);
    }
} 