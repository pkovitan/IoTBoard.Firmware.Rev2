#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include "Arduino.h"

class GPSModuleClass {
private:
    HardwareSerial* _serial;
    char _sim_ack[256]; // Buffer for SIM7600 responses
    
    // GPS data
    double _latitude;
    double _longitude;
    float _speed;
    float _direction;
    char _time[25]; // Format: "20YY-MM-DDThh:mm:ss.sssZ"
    float _altitude;
    
    // Status flags
    bool _gpsInitialized;
    bool _hasValidFix;
    unsigned long _lastPrintTime;
    
    // Parse GPS information from SIM7600 response
    bool parseGPS();

public:
    GPSModuleClass();
    
    // Initialize GPS module
    bool begin();
    
    // Update GPS data (call this regularly)
    void update();
    
    // Print GPS data to Serial
    void printData();
    
    // Getters for GPS data
    double getLatitude() const { return _latitude; }
    double getLongitude() const { return _longitude; }
    float getSpeed() const { return _speed; }
    float getDirection() const { return _direction; }
    float getAltitude() const { return _altitude; }
    const char* getTime() const { return _time; }
    bool hasValidFix() const { return _hasValidFix; }
};

// Global instance
extern GPSModuleClass GPSModule;

#endif // GPS_MODULE_H 