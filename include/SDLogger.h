#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

class SDLogger {
private:
    uint8_t _csPin;
    char _currentFilename[32];  // Buffer for the dynamic filename
    time_t _lastFileDay;        // Keep track of the current day
    bool _isInitialized;

    // Private method to get timestamp
    String getTimestamp();

public:
    // Pin definition as static constexpr member
    static constexpr uint8_t DEFAULT_CS_PIN = 5;

    // Constructor with default CS pin
    SDLogger(uint8_t csPin = DEFAULT_CS_PIN);
    
    // Initialize the SD card
    bool begin();
    
    // Log methods with different severity levels
    bool logInfo(const char* message);
    bool logWarning(const char* message);
    bool logError(const char* message);
    
    // Check if SD card is available
    bool isAvailable() const { return _isInitialized; }

    const char* getCurrentFilename() const { return _currentFilename; }

private:
    void updateFilename();      // Updates filename based on current date
    bool checkNewDay();         // Check if we need to create a new file
};

#endif // SD_LOGGER_H 