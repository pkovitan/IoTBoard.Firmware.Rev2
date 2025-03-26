#pragma once

#include <Arduino.h>
#include "config.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class SDLoggerClass {
public:
    SDLoggerClass();
    
    // Initialization
    bool begin(uint8_t csPin = 5);
    
    // Write log to SD card
    bool writeLog(const char* payload);
    
    // Read log from SD card
    bool readLog(const char* filename, void (*callback)(const char* line));
    
    // Get status
    bool isCardMounted();
    
    // Get statistics
    uint64_t getTotalBytes();
    uint64_t getUsedBytes();
    uint64_t getFreeBytes();
    
    // Get current log file name
    const char* getCurrentFileName();
    
    // Set custom file name format (default: "/YYYY-MM-DD.txt")
    void setFileNameFormat(const char* format);
    
    // Update file name based on current date/time
    void updateFileName(const char* timeString);
    
    // Set maximum file size (in bytes)
    void setMaxFileSize(uint64_t size);
    
    // Set minimum free space required (in bytes)
    void setMinFreeSpace(uint64_t size);
    
    // Format SD card (delete all files)
    bool formatSDCard();
    
    // Periodic update
    void update();
    
private:
    // SD card status
    bool cardMounted;
    
    // SD card pin
    uint8_t csPin;
    
    // Current log file name
    char currentFileName[32];
    
    // File name format
    char fileNameFormat[32];
    
    // File handle
    File logFile;
    
    // Statistics
    uint64_t totalBytes;
    uint64_t usedBytes;
    uint64_t freeBytes;
    
    // File management
    uint64_t maxFileSize;      // Maximum size of each log file
    uint64_t minFreeSpace;     // Minimum free space required
    uint64_t currentFileSize;  // Current size of the log file
    
    // Update card statistics
    void updateStatistics();
    void updateStats();
    
    // File management
    bool checkFileSize();
    bool checkFreeSpace();
    bool rotateFile();
    
    // Helper function to recursively delete directory contents
    bool deleteDirectory(const String& path);
};

// Global instance
extern SDLoggerClass SDLogger; 