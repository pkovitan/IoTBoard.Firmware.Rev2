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
    freeBytes = 0;
    
    // Default file management settings
    maxFileSize = 10 * 1024 * 1024;  // 10MB per file
    minFreeSpace = 100 * 1024 * 1024; // 100MB minimum free space
    currentFileSize = 0;
}

bool SDCardLoggerClass::begin(uint8_t csPin) {
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
    Serial.printf("Total space: %llu MB\n", totalBytes / (1024 * 1024));
    Serial.printf("Used space: %llu MB\n", usedBytes / (1024 * 1024));
    Serial.printf("Free space: %llu MB\n", freeBytes / (1024 * 1024));
    
    return true;
}

bool SDCardLoggerClass::writeLog(const char* payload) {
    if (!cardMounted) {
        Serial.println("SD Card not mounted. Cannot write log.");
        return false;
    }
    
    // Check if we need to rotate the file
    if (checkFileSize() || checkFreeSpace()) {
        if (!rotateFile()) {
            Serial.println("Failed to rotate log file");
            return false;
        }
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
        currentFileSize += bytesWritten;
        Serial.printf("Log written to %s (%d bytes)\n", currentFileName, bytesWritten);
        
        // Update statistics
        updateStatistics();
        
        return true;
    } else {
        Serial.println("Write failed");
        return false;
    }
}

bool SDCardLoggerClass::readLog(const char* filename, void (*callback)(const char* line)) {
    if (!cardMounted) {
        Serial.println("SD Card not mounted. Cannot read log.");
        return false;
    }
    
    // Open file for reading
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.printf("Failed to open file %s for reading\n", filename);
        return false;
    }
    
    // Read file line by line
    char line[256];
    while (file.available()) {
        int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
        if (len > 0) {
            line[len] = '\0';  // Null terminate the string
            callback(line);    // Call the callback function with the line
        }
    }
    
    file.close();
    return true;
}

bool SDCardLoggerClass::isCardMounted() {
    return cardMounted;
}

uint64_t SDCardLoggerClass::getTotalBytes() {
    return totalBytes;
}

uint64_t SDCardLoggerClass::getUsedBytes() {
    return usedBytes;
}

uint64_t SDCardLoggerClass::getFreeBytes() {
    return freeBytes;
}

const char* SDCardLoggerClass::getCurrentFileName() {
    return currentFileName;
}

void SDCardLoggerClass::setFileNameFormat(const char* format) {
    strcpy(fileNameFormat, format);
}

void SDCardLoggerClass::setMaxFileSize(uint64_t size) {
    maxFileSize = size;
}

void SDCardLoggerClass::setMinFreeSpace(uint64_t size) {
    minFreeSpace = size;
}

void SDCardLoggerClass::updateFileName(const char* timeString) {
    // Format: "2021-12-31T13:00:00.000Z"
    // Extract date part for filename
    char dateStr[11]; // YYYY-MM-DD\0
    strncpy(dateStr, timeString, 10);
    dateStr[10] = '\0';
    
    // Create filename
    sprintf(currentFileName, "/%s.txt", dateStr);
    
    // Reset current file size
    currentFileSize = 0;
    
    Serial.printf("Current log file: %s\n", currentFileName);
}

void SDCardLoggerClass::updateStatistics() {
    if (cardMounted) {
        totalBytes = SD.totalBytes();
        usedBytes = SD.usedBytes();
        freeBytes = totalBytes - usedBytes;
    }
}

bool SDCardLoggerClass::checkFileSize() {
    return currentFileSize >= maxFileSize;
}

bool SDCardLoggerClass::checkFreeSpace() {
    return freeBytes < minFreeSpace;
}

bool SDCardLoggerClass::rotateFile() {
    // Get current file info
    File file = SD.open(currentFileName, FILE_READ);
    if (!file) {
        Serial.println("Failed to open current file for rotation");
        return false;
    }
    
    // Create new filename with timestamp
    char newFileName[32];
    char timestamp[20];
    sprintf(timestamp, "%lu", millis());
    sprintf(newFileName, "%s.%s", currentFileName, timestamp);
    
    // Create new file
    File newFile = SD.open(newFileName, FILE_WRITE);
    if (!newFile) {
        Serial.println("Failed to create new file for rotation");
        file.close();
        return false;
    }
    
    // Copy content to new file
    while (file.available()) {
        newFile.write(file.read());
    }
    
    // Close both files
    file.close();
    newFile.close();
    
    // Delete old file
    if (!SD.remove(currentFileName)) {
        Serial.println("Failed to remove old file");
        return false;
    }
    
    // Reset current file size
    currentFileSize = 0;
    
    Serial.printf("Rotated log file to: %s\n", newFileName);
    return true;
}

void SDCardLoggerClass::update() {
    // Add any periodic update logic here if needed
} 