#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include "SensorManager.h"

// Vehicle event types
enum VehicleEventType {
    NO_EVENT = 0,
    ROLLOVER_EVENT = 1,        // Vehicle rollover/flip
    HARD_BRAKE_EVENT = 2,      // Sudden braking
    LANE_CHANGE_EVENT = 3,     // Sudden lane change
    RAPID_ACCEL_EVENT = 4,     // Rapid acceleration
    SPIN_EVENT = 5,            // Vehicle spin
    HIGH_VIBRATION_EVENT = 6   // Rough road/high vibration
};

// Axis orientation enum
enum AxisOrientation {
    FORWARD_X_UP_Z = 0,        // Standard orientation
    FORWARD_Y_UP_Z = 1,        // 90° CW rotation around Z
    FORWARD_NEG_X_UP_Z = 2,    // 180° rotation around Z
    FORWARD_NEG_Y_UP_Z = 3,    // 90° CCW rotation around Z
    CUSTOM_ORIENTATION = 99    // Custom orientation after calibration
};

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
    
    // Get corrected values (after axis calibration)
    float getCorrectedAccX();
    float getCorrectedAccY();
    float getCorrectedAccZ();
    float getCorrectedGyroX();
    float getCorrectedGyroY();
    float getCorrectedGyroZ();
    
    // Main update loop - should be called in main loop
    void update();
    
    // Event callbacks
    void onMotionDetected(void (*callback)(float, float, float, float, float, float));
    void onVehicleEvent(void (*callback)(VehicleEventType, float, float, float, float, float, float));
    
    // Set motion threshold for callback
    void setThreshold(float threshold);
    
    // Vehicle event detection thresholds
    void setRolloverThreshold(float threshold);
    void setHardBrakeThreshold(float threshold);
    void setLaneChangeThreshold(float threshold);
    void setRapidAccelThreshold(float threshold);
    void setSpinThreshold(float threshold);
    void setVibrationThreshold(float threshold);
    
    // Check for vehicle events
    VehicleEventType detectVehicleEvent();
    bool isRollover();
    bool isHardBrake();
    bool isLaneChange();
    bool isRapidAcceleration();
    bool isSpinning();
    bool isHighVibration();
    
    // Axis calibration
    void calibrateZAxis();
    void calibrateXYAxes(float speed);
    bool isCalibrated();
    void setAxisOrientation(AxisOrientation orientation);
    AxisOrientation getAxisOrientation();
    
    // Default event handlers that can be used in main
    void processVehicleEvent(VehicleEventType eventType, float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);
    void defaultMotionHandler(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ);

private:
    LSM6DS3 imu;
    
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float lastAccX, lastAccY, lastAccZ;
    float lastGyroX, lastGyroY, lastGyroZ;
    float motionThreshold;
    unsigned long lastReadTime;
    
    // Corrected (calibrated) values
    float corrAccX, corrAccY, corrAccZ;
    float corrGyroX, corrGyroY, corrGyroZ;
    
    // Calibration matrices
    float rotationMatrix[3][3];
    bool calibrated;
    AxisOrientation axisOrientation;
    
    // Calibration reference values
    float gravityVector[3];
    float forwardVector[3];
    
    // Vehicle event thresholds
    float rolloverThreshold;
    float hardBrakeThreshold;
    float laneChangeThreshold;
    float rapidAccelThreshold;
    float spinThreshold;
    float vibrationThreshold;
    
    // Event window data collection
    static const int EVENT_WINDOW_SIZE = 20;
    float accXWindow[EVENT_WINDOW_SIZE];
    float accYWindow[EVENT_WINDOW_SIZE];
    float accZWindow[EVENT_WINDOW_SIZE];
    float gyroXWindow[EVENT_WINDOW_SIZE];
    float gyroYWindow[EVENT_WINDOW_SIZE];
    float gyroZWindow[EVENT_WINDOW_SIZE];
    int windowIndex;
    
    // Event detection state
    VehicleEventType lastEventType;
    unsigned long lastEventTime;
    
    // Callbacks
    void (*motionDetectedCallback)(float, float, float, float, float, float);
    void (*vehicleEventCallback)(VehicleEventType, float, float, float, float, float, float);
    
    // Apply calibration to raw values
    void applyCalibration();
    
    // Update event window with new values
    void updateEventWindow();
    
    // Read interval in milliseconds (changed from 50ms to 20ms for much better event detection)
    static const unsigned long READ_INTERVAL = 20; // Read every 20 milliseconds (50Hz)
    
    // Minimum time between reported events (to prevent duplicate events)
    static const unsigned long MIN_EVENT_INTERVAL = 5000; // 5 seconds
};

// Global instance
extern AccelerometerGyroClass AccelerometerGyro; 