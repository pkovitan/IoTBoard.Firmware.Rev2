#include "SensorManager.h"
#include <time.h>

// Create global instance
SensorManagerClass SensorManager;

SensorManagerClass::SensorManagerClass() {
    // Initialize default values
    strcpy(fleet, "WillDeleteLater");
    strcpy(id, "Vehicle01");
    seq = 0;
    strncpy(timeString, "2000-01-01T00:00:00.000Z", sizeof(timeString) - 1);
    timeString[sizeof(timeString) - 1] = '\0';
    strcpy(event, "NONE");
    strcpy(cardReader, "00000000");
    vin = 0.0;
    vbat = 0.0;
    strcpy(engine, "OFF");
    speed = 0.0;
    direction = 0.0;
    fuel = 0.0;
    latitude = 0.0;
    longitude = 0.0;
    strcpy(door, "CLOSE");
    temp = 0.0;
    accX = 0.0;
    accY = 0.0;
    accZ = 0.0;
    gyrX = 0.0;
    gyrY = 0.0;
    gyrZ = 0.0;
    strcpy(emergency, "FALSE");
    
    // Initialize payload buffers
    memset(jsonPayload, 0, sizeof(jsonPayload));
    memset(nmeaPayload, 0, sizeof(nmeaPayload));
    
    // Initialize auto print settings
    autoPrintEnabled = true;
    printInterval = 1000; // 1 second default
    lastPrintTime = 0;
    autoPrintFormat = JSON_FORMAT;
    
    // Initialize event tracking
    lastEventTime = 0;
    eventResetInterval = 5000; // 5 seconds default
    
    // Update payloads
    updateJsonPayload();
    updateNmeaPayload();
}

bool SensorManagerClass::begin() {
    Serial.println("SensorManager initialized successfully");
    return true;
}

void SensorManagerClass::update() {
    // Read data from all sensors
    readAllSensors();
    
    // Update time string
    updateTimeString();
    
    // Check if we need to reset the event field
    unsigned long currentTime = millis();
    if (strcmp(event, "NONE") != 0 && (currentTime - lastEventTime >= eventResetInterval)) {
        Serial.println("Resetting event to NONE");
        strcpy(event, "NONE");
    }
    
    // Update payload strings
    updateJsonPayload();
    updateNmeaPayload();
    
    // Check if it's time to print payload
    if (autoPrintEnabled && (currentTime - lastPrintTime >= printInterval)) {
        lastPrintTime = currentTime;
        printPayload(autoPrintFormat);
    }
}

void SensorManagerClass::readAllSensors() {
    // Read card reader data
    strcpy(cardReader, CardReader.getCardID());
    
    // Read voltage data
    vin = VoltageMonitor.getVinVoltage();
    vbat = VoltageMonitor.getBatteryVoltage();
    
    // Read engine state
    strcpy(engine, EngineSensor.isOn() ? "ON" : "OFF");
    
    // Read GPS data
    if (GPSModule.hasValidFix()) {
        speed = GPSModule.getSpeed();
        direction = GPSModule.getDirection();
        latitude = GPSModule.getLatitude();
        longitude = GPSModule.getLongitude();
    }
    
    // Read fuel level
    fuel = FuelSensor.getFuelLevel();
    
    // Read door state
    strcpy(door, DoorSensor.isOpen() ? "OPEN" : "CLOSE");
    
    // Read temperature
    temp = TemperatureSensor.getTemperature();
    
    // Read accelerometer data
    accX = AccelerometerGyro.getAccX();
    accY = AccelerometerGyro.getAccY();
    accZ = AccelerometerGyro.getAccZ();
    
    // Read gyroscope data
    gyrX = AccelerometerGyro.getGyroX();
    gyrY = AccelerometerGyro.getGyroY();
    gyrZ = AccelerometerGyro.getGyroZ();
    
    // Read emergency state
    strcpy(emergency, EmergencyButton.isPressed() ? "TRUE" : "FALSE");
}

const char* SensorManagerClass::getPayload(PayloadFormat format) {
    if (format == JSON_FORMAT) {
        return jsonPayload;
    } else {
        return nmeaPayload;
    }
}

void SensorManagerClass::printPayload(PayloadFormat format) {
    Serial.println(getPayload(format));
}

void SensorManagerClass::enableAutoPrint(bool enable) {
    autoPrintEnabled = enable;
}

void SensorManagerClass::setPrintInterval(unsigned long interval) {
    printInterval = interval;
}

void SensorManagerClass::setFleet(const char* fleet) {
    strcpy(this->fleet, fleet);
}

void SensorManagerClass::setID(const char* id) {
    strcpy(this->id, id);
}

void SensorManagerClass::setEvent(const char* event) {
    strcpy(this->event, event);
    lastEventTime = millis(); // Record the time when the event was set
    Serial.print("Event set to: ");
    Serial.println(event);
}

void SensorManagerClass::setEventResetInterval(unsigned long interval) {
    eventResetInterval = interval;
}

void SensorManagerClass::updateJsonPayload() {
    // Format payload as JSON string
    sprintf(jsonPayload, 
        "{\"fleet\":\"%s\",\"id\":\"%s\",\"seq\":%d,\"time\":\"%s\",\"event\":\"%s\",\"card-reader\":\"%s\",\"vin\":%.1f,\"vbat\":%.1f,\"engine\":\"%s\",\"speed\":%.1f,\"direction\":%.1f,\"fuel\":%.1f,\"latitude\":%.7f,\"longitude\":%.7f,\"door\":\"%s\",\"temp\":%.1f,\"acc-x\":%.7f,\"acc-y\":%.7f,\"acc-z\":%.7f,\"gyr-x\":%.7f,\"gyr-y\":%.7f,\"gyr-z\":%.7f,\"emergency\":\"%s\"}",
        fleet, id, seq, timeString, event, cardReader, vin, vbat, engine, speed, direction, fuel, 
        latitude, longitude, door, temp, accX, accY, accZ, gyrX, gyrY, gyrZ, emergency
    );
}

void SensorManagerClass::updateNmeaPayload() {
    // Format payload as NMEA-0183 string
    // We'll use a custom sentence format: $PSENS,<fields>*<checksum>
    
    // First, create the data part of the sentence
    char nmeaSentence[900];
    sprintf(nmeaSentence, 
        "$PSENS,%s,%s,%d,%s,%s,%s,%.1f,%.1f,%s,%.1f,%.1f,%.1f,%.7f,%.7f,%s,%.1f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%s",
        fleet, id, seq++, timeString, event, cardReader, vin, vbat, engine, speed, direction, fuel, 
        latitude, longitude, door, temp, accX, accY, accZ, gyrX, gyrY, gyrZ, emergency
    );
    
    // Calculate checksum
    uint8_t checksum = calculateNmeaChecksum(nmeaSentence + 1); // Skip the '$'
    
    // Create the final NMEA sentence with checksum
    sprintf(nmeaPayload, "%s*%02X\r\n", nmeaSentence, checksum);
}

uint8_t SensorManagerClass::calculateNmeaChecksum(const char* sentence) {
    uint8_t checksum = 0;
    // XOR each character
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}

void SensorManagerClass::updateTimeString() {
    // Check if we have a valid GPS fix and time
    if (GPSModule.hasValidFix() && GPSModule.hasValidTime()) {
        // Use GPS time
        strcpy(timeString, GPSModule.getTime());
    } else {
        // Format: "2021-12-31T13:00:00.000Z"
        // For demonstration, we'll just use the millis() to update seconds
        unsigned long ms = millis() % 1000;
        unsigned long s = (millis() / 1000) % 60;
        unsigned long m = (millis() / 60000) % 60;
        unsigned long h = (millis() / 3600000) % 24;
        
        // This is just a placeholder. In a real application, you would use actual date/time
        sprintf(timeString, "2023-01-01T%02lu:%02lu:%02lu.%03luZ", h, m, s, ms);
    }
} 