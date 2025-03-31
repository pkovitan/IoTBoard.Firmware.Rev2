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

// Filter method enum
enum FilterMethod {
    SIMPLE_AVERAGE = 0,  // Simple moving average
    WEIGHTED_AVERAGE = 1, // More weight to recent values
    EXP_SMOOTHING = 2     // Exponential smoothing
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
    
    // Get smoothed values (after sliding window filtering)
    float getSmoothedAccX();
    float getSmoothedAccY();
    float getSmoothedAccZ();
    float getSmoothedGyroX();
    float getSmoothedGyroY();
    float getSmoothedGyroZ();
    
    // Get standard deviation values (for anomaly detection)
    float getAccStdDevX() { return accStdDevX; }
    float getAccStdDevY() { return accStdDevY; }
    float getAccStdDevZ() { return accStdDevZ; }
    float getGyroStdDevX() { return gyroStdDevX; }
    float getGyroStdDevY() { return gyroStdDevY; }
    float getGyroStdDevZ() { return gyroStdDevZ; }
    
    // Anomaly detection helper
    bool isAnomalyDetected(float stdDevThreshold);
    
    // Filter settings
    void enableFilter(bool enable);
    bool isFilterEnabled();
    void setFilterMethod(FilterMethod method);
    void setFilterAlpha(float alpha);
    
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
    
    // Smoothed (filtered) values
    float smoothAccX, smoothAccY, smoothAccZ;
    float smoothGyroX, smoothGyroY, smoothGyroZ;
    
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
    
    // Sliding Window Filter data
    static const int FILTER_WINDOW_SIZE = 10;
    float accXFilterWindow[FILTER_WINDOW_SIZE];
    float accYFilterWindow[FILTER_WINDOW_SIZE];
    float accZFilterWindow[FILTER_WINDOW_SIZE];
    float gyroXFilterWindow[FILTER_WINDOW_SIZE];
    float gyroYFilterWindow[FILTER_WINDOW_SIZE];
    float gyroZFilterWindow[FILTER_WINDOW_SIZE];
    int filterWindowIndex;
    bool filterEnabled;
    
    // Filter method
    FilterMethod filterMethod;
    float filterAlpha; // Alpha parameter for exponential smoothing
    
    // Standard deviation for anomaly detection
    float accStdDevX, accStdDevY, accStdDevZ;
    float gyroStdDevX, gyroStdDevY, gyroStdDevZ;
    
    // Event detection state
    VehicleEventType lastEventType;
    unsigned long lastEventTime;
    
    // Callbacks
    void (*motionDetectedCallback)(float, float, float, float, float, float);
    void (*vehicleEventCallback)(VehicleEventType, float, float, float, float, float, float);
    
    // Apply calibration to raw values
    void applyCalibration();
    
    // Apply sliding window filter to values
    void applyFilter();
    
    // Update event window with new values
    void updateEventWindow();
    
    // Update filter window with new values
    void updateFilterWindow();
    
    // Calculate moving average for filtered values
    float calculateMovingAverage(float* window, int size);
    
    // Calculate weighted moving average
    float calculateWeightedAverage(float* window, int size);
    
    // Calculate standard deviation
    float calculateStandardDeviation(float* window, int size, float average);
    
    // Apply exponential smoothing
    float applyExponentialSmoothing(float newValue, float prevSmoothed, float alpha);
    
    // Read interval in milliseconds (changed from 50ms to 20ms for much better event detection)
    static const unsigned long READ_INTERVAL = 20; // Read every 20 milliseconds (50Hz)
    
    // Minimum time between reported events (to prevent duplicate events)
    static const unsigned long MIN_EVENT_INTERVAL = 5000; // 5 seconds
};

// Global instance
extern AccelerometerGyroClass AccelerometerGyro; 