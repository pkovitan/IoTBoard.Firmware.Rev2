#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <SparkFunLSM6DS3.h>

class AccelerometerGyroClass {
public:
    AccelerometerGyroClass();
    
    // Initialization
    bool begin();
    
    // Get accelerometer values (in g)
    float getAccX();
    float getAccY();
    float getAccZ();
    
    // Get gyroscope values (in degrees per second)
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onMotionDetected(void (*callback)(float, float, float, float, float, float));
    
    // Set motion threshold for callback
    void setThreshold(float threshold);

private:
    LSM6DS3 imu;
    
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float lastAccX, lastAccY, lastAccZ;
    float motionThreshold;
    unsigned long lastReadTime;
    
    void (*motionDetectedCallback)(float, float, float, float, float, float);
    
    // Read interval in milliseconds
    static const unsigned long READ_INTERVAL = 1000; // Read every 1 second
};

// Global instance
extern AccelerometerGyroClass AccelerometerGyro; 