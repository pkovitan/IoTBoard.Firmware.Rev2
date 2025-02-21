#include <Arduino.h>
#include "SDLogger.h"
#include "LEDIndicator.h"
#include <PCF8574.h>
#include <WiFi.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 28800;     // Change this based on your timezone (e.g., 28800 for GMT+8)
const int   daylightOffset_sec = 0;    // Change if needed for daylight saving

// Create instances
SDLogger logger;
PCF8574 ex_io(0x20);
LEDIndicator leds(ex_io);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32 Node32s Starting...");

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Wait for time to be set
    struct tm timeinfo;
    while(!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        delay(1000);
    }

    // Initialize PCF8574
    if (!ex_io.begin()) {
        Serial.println("Failed to initialize PCF8574");
        while(1);
    }

    // Initialize LED indicator
    leds.begin();

    // Initialize SD Logger
    if (logger.begin()) {
        logger.logInfo("System started successfully");
        // leds.setRed(true);  // Indicate successful initialization
    } else {
        Serial.println("Failed to initialize SD Logger");
        // leds.setRed(true, true);  // Blink red to indicate error
    }

    // Disconnect WiFi if not needed after time sync
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void loop() {
    // Log a message every second
    logger.logInfo("System running");
    
    // Update LED states
    leds.loop();
    
    delay(1000);
} 