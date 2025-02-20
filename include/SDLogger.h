#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

class SDLogger {
private:
    const char* _logFileName;
    uint8_t _csPin;
    bool _isInitialized;

    // Private method to get timestamp
    String getTimestamp();

public:
    SDLogger(const char* logFileName = "/log.txt", uint8_t csPin = 5);
    
    // Initialize the SD card
    bool begin();
    
    // Log methods with different severity levels
    bool logInfo(const char* message);
    bool logWarning(const char* message);
    bool logError(const char* message);
    
    // Check if SD card is available
    bool isAvailable() const { return _isInitialized; }
};

#endif // SD_LOGGER_H 