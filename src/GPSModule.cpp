#include "GPSModule.h"
#include "config.h"

// If SIM7600_SERIAL is not defined in config.h, define it here
#ifndef SIM7600_SERIAL
#define SIM7600_SERIAL Serial2
#endif

// Create global instance
GPSModuleClass GPSModule;

GPSModuleClass::GPSModuleClass() {
    latitude = 0.0;
    longitude = 0.0;
    speed = 0.0;
    direction = 0.0;
    strncpy(timeString, "2000-01-01T00:00:00.000Z", sizeof(timeString) - 1);
    timeString[sizeof(timeString) - 1] = '\0';
    validFix = false;
    lastLatitude = 0.0;
    lastLongitude = 0.0;
    locationThreshold = 0.0001; // Approximately 10 meters at the equator
    locationChangeCallback = nullptr;
    lastUpdateTime = 0;
}

bool GPSModuleClass::begin() {
    Serial.println("Initializing GPS module...");
    SIM7600_SERIAL.println("AT+CGPS=1");
    delay(1000);
    
    // Clear any pending data
    while (SIM7600_SERIAL.available()) {
        SIM7600_SERIAL.read();
    }
    
    Serial.println("GPS module initialized successfully");
    return true;
}

void GPSModuleClass::update() {
    unsigned long currentTime = millis();
    
    // Update GPS data at regular intervals
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentTime;
        
        // Request GPS data from SIM7600
        SIM7600_SERIAL.println("AT+CGPSINFO");
        delay(300); // Wait for response
        
        validFix = parseGPS();
        
        if (validFix) {
            // Print GPS data
            Serial.print("GPS: ");
            Serial.print(latitude, 7);
            Serial.print(", ");
            Serial.print(longitude, 7);
            Serial.print(", Speed: ");
            Serial.print(speed);
            Serial.print(" km/h, Direction: ");
            Serial.print(direction);
            Serial.print("°, Time: ");
            Serial.println(timeString);
            
            // Check if location has changed significantly
            float distance = calculateDistance(latitude, longitude, lastLatitude, lastLongitude);
            
            if (distance > locationThreshold * 111000) { // Convert degrees to meters (approx)
                // Call the callback if registered
                if (locationChangeCallback != nullptr) {
                    locationChangeCallback(latitude, longitude);
                }
                
                // Update last known position
                lastLatitude = latitude;
                lastLongitude = longitude;
            }
        } else {
            Serial.println("GPS: Waiting for valid fix...");
        }
    }
}

bool GPSModuleClass::parseGPS() {
    char sim_ack[256]; // Buffer for SIM7600 responses
    memset(sim_ack, 0, sizeof(sim_ack));
    unsigned long parse_time = millis();
    const unsigned long TIMEOUT = 1000; // 1 second timeout
    
    // Wait for incoming message
    while ((millis() - parse_time) < TIMEOUT) {
        if (SIM7600_SERIAL.available()) {
            SIM7600_SERIAL.readBytes(sim_ack, sizeof(sim_ack) - 1);
            
            // Parse GPS data from message
            if (strstr(sim_ack, "INFO:") != NULL) {
                char gps_lat[12], gps_log[13], gps_date[7], gps_time[9], gps_speed[6], gps_course[6];
                char* index_left;
                char* index_right;
                uint32_t len;
                
                // Check if we have valid data
                if (strstr(sim_ack, ",,,,") != NULL) {
                    return false;
                }
                
                // Latitude left boundary
                index_left = strchr(sim_ack, ':');
                if (index_left == NULL) {
                    return false;
                }
                
                // Latitude right boundary
                index_right = strchr(index_left, ',');
                if (index_right == NULL) {
                    return false;
                }
                
                len = index_right - index_left;
                if (len <= 1) {
                    return false;
                }
                
                memcpy(gps_lat, index_left + 1, len - 1);
                gps_lat[len - 1] = '\0';
                latitude = convertToDecimalDegrees(atof(gps_lat));
                
                // N/S indicator
                index_left = index_right + 1;
                char latDir = *index_left;
                if (latDir == 'S') {
                    latitude = -latitude;
                }
                
                // Longitude left boundary
                index_left = index_right + 3;
                
                // Longitude right boundary
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    return false;
                }
                
                memcpy(gps_log, index_left, len);
                gps_log[len] = '\0';
                longitude = convertToDecimalDegrees(atof(gps_log));
                
                // E/W indicator
                index_left = index_right + 1;
                char lonDir = *index_left;
                if (lonDir == 'W') {
                    longitude = -longitude;
                }
                
                // Date left boundary
                index_left = index_right + 3;
                
                // Date right boundary
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    return false;
                }
                
                memcpy(gps_date, index_left, len);
                gps_date[len] = '\0';
                uint32_t date_ddmmyy = atoi(gps_date);
                uint32_t date_dd = date_ddmmyy / 10000;
                uint32_t date_mm = (date_ddmmyy - (date_dd * 10000)) / 100;
                uint32_t date_yy = date_ddmmyy - ((date_ddmmyy / 100) * 100);
                
                // Time left boundary
                index_left = index_right + 1;
                
                // Time right boundary
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    return false;
                }
                
                memcpy(gps_time, index_left, len);
                gps_time[len] = '\0';
                float time_hhmmss = atof(gps_time);
                uint32_t time_hh = ((uint32_t)time_hhmmss) / 10000;
                uint32_t time_mm = (((uint32_t)time_hhmmss) - (time_hh * 10000)) / 100;
                float time_ss = time_hhmmss - (time_hh * 10000.0) - (time_mm * 100.0);
                
                // Set time
                sprintf(timeString, "20%02d-%02d-%02dT%02d:%02d:%06.3fZ", 
                        date_yy, date_mm, date_dd, time_hh, time_mm, time_ss);
                
                // Speed
                index_left = index_right + 1;
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    return false;
                }
                
                memcpy(gps_speed, index_left, len);
                gps_speed[len] = '\0';
                speed = 1.852 * atof(gps_speed); // 1 knot = 1.852 km/h
                
                // Course/Direction
                index_left = index_right + 1;
                index_right = strchr(index_left, '\r');
                if (index_right == NULL) {
                    index_right = strchr(index_left, '\0');
                }
                
                len = index_right - index_left;
                if (len < 1) {
                    return false;
                }
                
                memcpy(gps_course, index_left, len);
                gps_course[len] = '\0';
                direction = atof(gps_course);
                
                validFix = true;
                return true;
            }
        }
    }
    
    return false;
}

double GPSModuleClass::convertToDecimalDegrees(double nmeaValue) {
    int degrees = (int)(nmeaValue / 100);
    double minutes = nmeaValue - (degrees * 100);
    return degrees + (minutes / 60.0);
}

double GPSModuleClass::getLatitude() {
    return latitude;
}

double GPSModuleClass::getLongitude() {
    return longitude;
}

float GPSModuleClass::getSpeed() {
    return speed;
}

float GPSModuleClass::getDirection() {
    return direction;
}

const char* GPSModuleClass::getTime() {
    return timeString;
}

bool GPSModuleClass::hasValidFix() {
    return validFix;
}

void GPSModuleClass::onLocationChange(void (*callback)(double lat, double lon)) {
    locationChangeCallback = callback;
}

void GPSModuleClass::setThreshold(float threshold) {
    locationThreshold = threshold;
}

float GPSModuleClass::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Simple approximation for small distances
    // 1 degree of latitude = ~111 km
    // 1 degree of longitude = ~111 km * cos(latitude)
    double latDiff = lat2 - lat1;
    double lonDiff = lon2 - lon1;
    double latDistance = latDiff * 111000; // Convert to meters
    double lonDistance = lonDiff * 111000 * cos(lat1 * M_PI / 180.0); // Convert to meters
    
    return sqrt(latDistance * latDistance + lonDistance * lonDistance);
}

// Initialize the GPS module
void setupGPS() {
    GPSModule.begin();
}

// Update GPS data - call this in the main loop
void updateGPS() {
    GPSModule.update();
} 