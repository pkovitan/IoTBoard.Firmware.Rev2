#pragma once

#include <Arduino.h>
#include "config.h"

class GPSModuleClass {
public:
    GPSModuleClass();
    
    // Initialization
    bool begin();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Get GPS data
    double getLatitude();
    double getLongitude();
    float getSpeed();
    float getDirection();
    const char* getTime();
    
    // Check if GPS has a valid fix
    bool hasValidFix();
    
    // Check if GPS has valid time (same as hasValidFix for now)
    bool hasValidTime() { return hasValidFix(); }
    
    // Event callbacks
    void onLocationChange(void (*callback)(double lat, double lon));
    
    // Set distance threshold for callback (in meters)
    void setThreshold(float threshold);

private:
    // GPS data
    double latitude;
    double longitude;
    float speed;
    float direction;
    char timeString[24]; // Format: "2021-12-31T13:00:00.000Z"
    bool validFix;
    
    // Previous values for change detection
    double lastLatitude;
    double lastLongitude;
    
    // Threshold for location change callback (in degrees)
    float locationThreshold;
    
    // Callback function
    void (*locationChangeCallback)(double lat, double lon);
    
    // Last update time
    unsigned long lastUpdateTime;
    
    // Update interval in milliseconds
    static const unsigned long UPDATE_INTERVAL = 1000; // 1 second
    
    // Parse GPS data from SIM7600
    bool parseGPS();
    
    // Calculate distance between two GPS coordinates (in meters)
    float calculateDistance(double lat1, double lon1, double lat2, double lon2);
    
    // Convert NMEA format (DDMM.MMMM) to decimal degrees
    double convertToDecimalDegrees(double nmeaValue);
};

// Global instance
extern GPSModuleClass GPSModule; 