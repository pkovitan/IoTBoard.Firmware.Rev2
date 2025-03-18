#include "GPSModule.h"
#include "config.h"

// If SIM7600_SERIAL is not defined in config.h, define it here
#ifndef SIM7600_SERIAL
#define SIM7600_SERIAL Serial2
#endif

// Create global instance
GPSModuleClass GPSModule;

// Define payload variables here instead of using external ones
double payload_latitude = 0.0;
double payload_longitude = 0.0;
float payload_speed = 0.0;
float payload_direction = 0.0;
char payload_time[25] = "2000-01-01T00:00:00.000Z";

GPSModuleClass::GPSModuleClass() : 
    _serial(&SIM7600_SERIAL),
    _latitude(0.0),
    _longitude(0.0),
    _speed(0.0),
    _direction(0.0),
    _altitude(0.0),
    _gpsInitialized(false),
    _hasValidFix(false),
    _lastPrintTime(0) {
    
    memset(_time, 0, sizeof(_time));
    strcpy(_time, "2000-01-01T00:00:00.000Z");
}

bool GPSModuleClass::begin() {
    Serial.println("Initializing GPS module...");
    _serial->println("AT+CGPS=1");
    delay(1000);
    
    // Clear any pending data
    while (_serial->available()) {
        _serial->read();
    }
    
    _gpsInitialized = true;
    Serial.println("GPS module initialized successfully");
    return true;
}

void GPSModuleClass::update() {
    if (!_gpsInitialized) {
        return;
    }
    
    _serial->println("AT+CGPSINFO");
    delay(300); // Wait for response
    
    bool result = parseGPS();
    
    // Update payload variables if we have a valid fix
    if (result) {
        payload_latitude = _latitude;
        payload_longitude = _longitude;
        payload_speed = _speed;
        payload_direction = _direction;
        strcpy(payload_time, _time);
    }
    
    // Print GPS data every second
    unsigned long currentTime = millis();
    if (currentTime - _lastPrintTime >= 1000) { // 1 second interval
        _lastPrintTime = currentTime;
        printData();
    }
}

bool GPSModuleClass::parseGPS() {
    memset(_sim_ack, 0, sizeof(_sim_ack));
    unsigned long parse_time = millis();
    const unsigned long TIMEOUT = 1000; // 1 second timeout
    
    // Wait for incoming message
    while ((millis() - parse_time) < TIMEOUT) {
        if (_serial->available()) {
            _serial->readBytes(_sim_ack, sizeof(_sim_ack) - 1);
            
            // Parse GPS data from message
            if (strstr(_sim_ack, "INFO:") != NULL) {
                char gps_lat[12], gps_log[13], gps_date[7], gps_time[9], gps_speed[6], gps_course[6];
                char* index_left;
                char* index_right;
                uint32_t len;
                
                // Check if we have valid data
                if (strstr(_sim_ack, ",,,,") != NULL) {
                    _hasValidFix = false;
                    return false;
                }
                
                // Latitude left boundary
                index_left = strchr(_sim_ack, ':');
                if (index_left == NULL) {
                    _hasValidFix = false;
                    return false;
                }
                
                // Latitude right boundary
                index_right = strchr(index_left, ',');
                if (index_right == NULL) {
                    _hasValidFix = false;
                    return false;
                }
                
                len = index_right - index_left;
                if (len <= 1) {
                    _hasValidFix = false;
                    return false;
                }
                
                memcpy(gps_lat, index_left + 1, len - 1);
                gps_lat[len - 1] = '\0';
                double lat_ddmm = atof(gps_lat);
                double lat_dd = (double)(((int)lat_ddmm) / 100);
                double lat_mm = (lat_ddmm - (lat_dd * 100)) / 60;
                _latitude = lat_dd + lat_mm;
                
                // Check N/S
                index_left = index_right + 1;
                if (*index_left == 'S') {
                    _latitude = -_latitude;
                }
                
                // Longitude left boundary
                index_left = index_right + 3;
                
                // Longitude right boundary
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    _hasValidFix = false;
                    return false;
                }
                
                memcpy(gps_log, index_left, len);
                gps_log[len] = '\0';
                double log_dddmm = atof(gps_log);
                double log_ddd = (double)(((int)log_dddmm) / 100);
                double log_mm = (log_dddmm - (log_ddd * 100)) / 60;
                _longitude = log_ddd + log_mm;
                
                // Check E/W
                index_left = index_right + 1;
                if (*index_left == 'W') {
                    _longitude = -_longitude;
                }
                
                // Date left boundary
                index_left = index_right + 3;
                
                // Date right boundary
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    _hasValidFix = false;
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
                    _hasValidFix = false;
                    return false;
                }
                
                memcpy(gps_time, index_left, len);
                gps_time[len] = '\0';
                float time_hhmmss = atof(gps_time);
                uint32_t time_hh = ((uint32_t)time_hhmmss) / 10000;
                uint32_t time_mm = (((uint32_t)time_hhmmss) - (time_hh * 10000)) / 100;
                float time_ss = time_hhmmss - (time_hh * 10000.0) - (time_mm * 100.0);
                
                // Set time
                sprintf(_time, "20%02d-%02d-%02dT%02d:%02d:%06.3fZ", 
                        date_yy, date_mm, date_dd, time_hh, time_mm, time_ss);
                
                // Altitude (if available)
                index_left = index_right + 1;
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len > 0) {
                    char gps_alt[10];
                    memcpy(gps_alt, index_left, len);
                    gps_alt[len] = '\0';
                    _altitude = atof(gps_alt);
                }
                
                // Speed
                index_left = index_right + 1;
                index_right = strchr(index_left, ',');
                len = index_right - index_left;
                if (len < 1) {
                    _hasValidFix = false;
                    return false;
                }
                
                memcpy(gps_speed, index_left, len);
                gps_speed[len] = '\0';
                // Convert from knots to km/h
                _speed = 1.852 * atof(gps_speed); // 1 knot = 1.852 km/h
                
                // Course/Direction
                index_left = index_right + 1;
                index_right = strchr(index_left, '\r');
                if (index_right == NULL) {
                    index_right = strchr(index_left, '\0');
                }
                
                len = index_right - index_left;
                if (len < 1) {
                    _hasValidFix = false;
                    return false;
                }
                
                memcpy(gps_course, index_left, len);
                gps_course[len] = '\0';
                _direction = atof(gps_course);
                
                _hasValidFix = true;
                return true;
            }
        }
    }
    
    return false;
}

void GPSModuleClass::printData() {
    if (_hasValidFix) {
        Serial.println("=== GPS Data ===");
        Serial.print("Time: ");
        Serial.println(_time);
        Serial.print("Position: ");
        Serial.print(_latitude, 7);
        Serial.print(", ");
        Serial.println(_longitude, 7);
        Serial.print("Speed: ");
        Serial.print(_speed);
        Serial.println(" km/h");
        Serial.print("Direction: ");
        Serial.print(_direction);
        Serial.println(" degrees");
        Serial.print("Altitude: ");
        Serial.print(_altitude);
        Serial.println(" meters");
        Serial.println("===============");
    } else {
        Serial.println("GPS: Waiting for valid fix...");
    }
}

// Initialize the GPS module
void setupGPS() {
    GPSModule.begin();
}

// Update GPS data - call this in the main loop
void updateGPS() {
    GPSModule.update();
} 