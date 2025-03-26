#include "SDLogger.h"

// Create global instance
SDLoggerClass SDLogger;

SDLoggerClass::SDLoggerClass() {
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

bool SDLoggerClass::begin(uint8_t csPin) {
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

bool SDLoggerClass::writeLog(const char* payload) {
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

bool SDLoggerClass::readLog(const char* filename, void (*callback)(const char* line)) {
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

bool SDLoggerClass::isCardMounted() {
    return cardMounted;
}

uint64_t SDLoggerClass::getTotalBytes() {
    return totalBytes;
}

uint64_t SDLoggerClass::getUsedBytes() {
    return usedBytes;
}

uint64_t SDLoggerClass::getFreeBytes() {
    return freeBytes;
}

const char* SDLoggerClass::getCurrentFileName() {
    return currentFileName;
}

void SDLoggerClass::setFileNameFormat(const char* format) {
    strcpy(fileNameFormat, format);
}

void SDLoggerClass::setMaxFileSize(uint64_t size) {
    maxFileSize = size;
}

void SDLoggerClass::setMinFreeSpace(uint64_t size) {
    minFreeSpace = size;
}

void SDLoggerClass::updateFileName(const char* timeString) {
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

void SDLoggerClass::updateStatistics() {
    if (cardMounted) {
        totalBytes = SD.totalBytes();
        usedBytes = SD.usedBytes();
        freeBytes = totalBytes - usedBytes;
    }
}

bool SDLoggerClass::checkFileSize() {
    return currentFileSize >= maxFileSize;
}

bool SDLoggerClass::checkFreeSpace() {
    return freeBytes < minFreeSpace;
}

bool SDLoggerClass::rotateFile() {
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

void SDLoggerClass::update() {
    // Add any periodic update logic here if needed
}

bool SDLoggerClass::formatSDCard() {
    if (!cardMounted) {
        Serial.println("Error: SD card not mounted");
        return false;
    }

    Serial.println("Starting SD card format...");
    Serial.println("Opening root directory...");

    // Open root directory
    File root = SD.open("/");
    if (!root) {
        Serial.println("Error: Failed to open root directory");
        return false;
    }

    Serial.println("Root directory opened successfully");
    bool allFilesDeleted = true;
    int fileCount = 0;
    
    // Delete all files in root directory
    Serial.println("Scanning root directory...");
    File file = root.openNextFile();
    while (file) {
        fileCount++;
        String filePath = String("/") + file.name();
        Serial.printf("Processing file %d: %s\n", fileCount, filePath.c_str());
        
        if (file.isDirectory()) {
            Serial.printf("Found directory: %s\n", filePath.c_str());
            // Recursively delete files in subdirectories
            if (!deleteDirectory(filePath)) {
                allFilesDeleted = false;
            }
        } else {
            Serial.printf("Deleting file: %s\n", filePath.c_str());
            // Delete file
            if (!SD.remove(filePath.c_str())) {
                Serial.printf("Failed to delete file: %s\n", filePath.c_str());
                allFilesDeleted = false;
            }
        }
        file = root.openNextFile();
    }

    // Close root directory
    root.close();
    Serial.printf("Processed %d files/directories\n", fileCount);

    // Update statistics
    updateStatistics();
    Serial.println("SD card statistics updated");

    if (allFilesDeleted) {
        Serial.println("SD card format completed successfully");
    } else {
        Serial.println("SD card format completed with some errors");
    }

    return allFilesDeleted;
}

// Helper function to recursively delete directory contents
bool SDLoggerClass::deleteDirectory(const String& path) {
    Serial.printf("Opening directory: %s\n", path.c_str());
    File dir = SD.open(path);
    if (!dir) {
        Serial.printf("Error: Failed to open directory: %s\n", path.c_str());
        return false;
    }

    bool allFilesDeleted = true;
    int fileCount = 0;
    
    while (File file = dir.openNextFile()) {
        fileCount++;
        String filePath = path + "/" + file.name();
        Serial.printf("Processing file %d in %s: %s\n", fileCount, path.c_str(), filePath.c_str());
        
        if (file.isDirectory()) {
            Serial.printf("Found subdirectory: %s\n", filePath.c_str());
            if (!deleteDirectory(filePath)) {
                allFilesDeleted = false;
            }
        } else {
            Serial.printf("Deleting file: %s\n", filePath.c_str());
            if (!SD.remove(filePath.c_str())) {
                Serial.printf("Failed to delete file: %s\n", filePath.c_str());
                allFilesDeleted = false;
            }
        }
    }
    
    dir.close();
    Serial.printf("Processed %d files in directory: %s\n", fileCount, path.c_str());

    return allFilesDeleted;
} 