#include "SDCardLogger.h"

// Create global instance
SDCardLoggerClass SDCardLogger;

SDCardLoggerClass::SDCardLoggerClass() {
    cardMounted = false;
    csPin = 5; // Default CS pin
    strcpy(currentFileName, "/log.txt");
    strcpy(fileNameFormat, "/%Y-%m-%d.txt");
    totalBytes = 0;
    usedBytes = 0;
}

bool SDCardLoggerClass::begin(int csPin) {
    this->csPin = csPin;
    
    Serial.print("Initializing SD card...");
    
    if (!SD.begin(csPin)) {
        Serial.println("Card Mount Failed!");
        cardMounted = false;
        return false;
    }
    
    cardMounted = true;
    Serial.println("Card Mounted Successfully");
    
    // Get card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        cardMounted = false;
        return false;
    }
    
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    
    // Update statistics
    updateStatistics();
    
    // Print card info
    Serial.printf("Total space: %lu MB\n", totalBytes / (1024 * 1024));
    Serial.printf("Used space: %lu MB\n", usedBytes / (1024 * 1024));
    Serial.printf("Free space: %lu MB\n", (totalBytes - usedBytes) / (1024 * 1024));
    
    return true;
}

bool SDCardLoggerClass::writeLog(const char* payload) {
    if (!cardMounted) {
        Serial.println("SD Card not mounted. Cannot write log.");
        return false;
    }
    
    // Open file for appending
    File file = SD.open(currentFileName, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return false;
    }
    
    // Write payload to file
    size_t bytesWritten = file.println(payload);
    file.close();
    
    if (bytesWritten > 0) {
        Serial.printf("Log written to %s (%d bytes)\n", currentFileName, bytesWritten);
        
        // Update statistics
        updateStatistics();
        
        return true;
    } else {
        Serial.println("Write failed");
        return false;
    }
}

bool SDCardLoggerClass::isCardMounted() {
    return cardMounted;
}

unsigned long SDCardLoggerClass::getTotalBytes() {
    return totalBytes;
}

unsigned long SDCardLoggerClass::getUsedBytes() {
    return usedBytes;
}

unsigned long SDCardLoggerClass::getFreeBytes() {
    return totalBytes - usedBytes;
}

const char* SDCardLoggerClass::getCurrentFileName() {
    return currentFileName;
}

void SDCardLoggerClass::setFileNameFormat(const char* format) {
    strcpy(fileNameFormat, format);
}

void SDCardLoggerClass::updateFileName(const char* timeString) {
    // Format: "2021-12-31T13:00:00.000Z"
    // Extract date part for filename
    char dateStr[11]; // YYYY-MM-DD\0
    strncpy(dateStr, timeString, 10);
    dateStr[10] = '\0';
    
    // Create filename
    sprintf(currentFileName, "/%s.txt", dateStr);
    
    Serial.printf("Current log file: %s\n", currentFileName);
}

void SDCardLoggerClass::updateStatistics() {
    if (cardMounted) {
        totalBytes = SD.totalBytes();
        usedBytes = SD.usedBytes();
    }
} 