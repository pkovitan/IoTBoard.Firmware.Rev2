#include <Arduino.h>
#include "SDLogger.h"

// Define SD card CS pin
#define SD_CS_PIN 5

// Create SDLogger instance
SDLogger logger("/datalog.txt", SD_CS_PIN);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32 Node32s Starting...");

    // Initialize SD Logger
    if (logger.begin()) {
        logger.logInfo("System started successfully");
    } else {
        Serial.println("Failed to initialize SD Logger");
    }
}

void loop() {
    // Log a message every second
    logger.logInfo("System running");
    delay(1000);
} 