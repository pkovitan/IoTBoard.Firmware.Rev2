#pragma once

#include <Arduino.h>
#include "config.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class SDCardLoggerClass {
public:
    SDCardLoggerClass();
    
    // Initialization
    bool begin(int csPin = 5);
    
    // Write log to SD card
    bool writeLog(const char* payload);
    
    // Get status
    bool isCardMounted();
    
    // Get statistics
    unsigned long getTotalBytes();
    unsigned long getUsedBytes();
    unsigned long getFreeBytes();
    
    // Get current log file name
    const char* getCurrentFileName();
    
    // Set custom file name format (default: "/YYYY-MM-DD.txt")
    void setFileNameFormat(const char* format);
    
    // Update file name based on current date/time
    void updateFileName(const char* timeString);
    
private:
    // SD card status
    bool cardMounted;
    
    // SD card pin
    int csPin;
    
    // Current log file name
    char currentFileName[32];
    
    // File name format
    char fileNameFormat[32];
    
    // Statistics
    unsigned long totalBytes;
    unsigned long usedBytes;
    
    // Update card statistics
    void updateStatistics();
};

// Global instance
extern SDCardLoggerClass SDCardLogger; 