#include "SDLogger.h"

SDLogger::SDLogger(uint8_t csPin)
    : _csPin(csPin)
    , _lastFileDay(0)
{
    _currentFilename[0] = '\0';  // Initialize empty string
}

bool SDLogger::begin() {
    if (!SD.begin(_csPin)) {
        return false;
    }
    updateFilename();
    return true;
}

void SDLogger::updateFilename() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        strcpy(_currentFilename, "/error.txt");
        return;
    }
    
    // Format: /YYYY-MM-DD-log.txt
    snprintf(_currentFilename, sizeof(_currentFilename),
             "/%04d-%02d-%02d-log.txt",
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday);
}

bool SDLogger::checkNewDay() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        return false;
    }
    
    // Convert to days since epoch
    time_t now = mktime(&timeinfo);
    time_t currentDay = now / (24*60*60);
    
    if (currentDay != _lastFileDay) {
        _lastFileDay = currentDay;
        updateFilename();
        return true;
    }
    return false;
}

void SDLogger::logInfo(const char* message) {
    // Check if we need to create a new file for a new day
    checkNewDay();
    
    // Open the file in append mode
    File file = SD.open(_currentFilename, FILE_APPEND);
    if (!file) {
        return;
    }
    
    // Get current time
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        file.close();
        return;
    }
    
    // Write timestamp and message
    char timestamp[20];
    snprintf(timestamp, sizeof(timestamp), "%02d:%02d:%02d", 
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);
             
    file.print(timestamp);
    file.print(" - ");
    file.println(message);
    file.close();
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