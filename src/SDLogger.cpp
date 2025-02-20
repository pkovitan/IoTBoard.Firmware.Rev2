#include "SDLogger.h"

SDLogger::SDLogger(const char* logFileName, uint8_t csPin) 
    : _logFileName(logFileName)
    , _csPin(csPin)
    , _isInitialized(false) {
}

bool SDLogger::begin() {
    if (!SD.begin(_csPin)) {
        Serial.println("SD Card initialization failed!");
        return false;
    }
    
    _isInitialized = true;
    Serial.println("SD Card initialized successfully");
    
    // Create header in log file if it doesn't exist
    File logFile = SD.open(_logFileName, FILE_APPEND);
    if (logFile) {
        logFile.println("Time,Level,Message");
        logFile.close();
        return true;
    }
    
    return false;
}

String SDLogger::getTimestamp() {
    unsigned long currentMillis = millis();
    unsigned long seconds = currentMillis / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    char timestamp[20];
    sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", 
        hours, 
        minutes % 60, 
        seconds % 60, 
        currentMillis % 1000);
    
    return String(timestamp);
}

bool SDLogger::logInfo(const char* message) {
    if (!_isInitialized) return false;
    
    File logFile = SD.open(_logFileName, FILE_APPEND);
    if (logFile) {
        logFile.printf("%s,INFO,%s\n", getTimestamp().c_str(), message);
        logFile.close();
        return true;
    }
    return false;
}

bool SDLogger::logWarning(const char* message) {
    if (!_isInitialized) return false;
    
    File logFile = SD.open(_logFileName, FILE_APPEND);
    if (logFile) {
        logFile.printf("%s,WARNING,%s\n", getTimestamp().c_str(), message);
        logFile.close();
        return true;
    }
    return false;
}

bool SDLogger::logError(const char* message) {
    if (!_isInitialized) return false;
    
    File logFile = SD.open(_logFileName, FILE_APPEND);
    if (logFile) {
        logFile.printf("%s,ERROR,%s\n", getTimestamp().c_str(), message);
        logFile.close();
        return true;
    }
    return false;
} 