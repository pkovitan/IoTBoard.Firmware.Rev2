#include "AccelerometerGyro.h"
#include <math.h>

// Create global instance
AccelerometerGyroClass AccelerometerGyro;

AccelerometerGyroClass::AccelerometerGyroClass() : imu(I2C_MODE, 0x6B) {
    // Initialize raw values
    accX = accY = accZ = 0.0;
    gyroX = gyroY = gyroZ = 0.0;
    lastAccX = lastAccY = lastAccZ = 0.0;
    lastGyroX = lastGyroY = lastGyroZ = 0.0;
    
    // Initialize corrected values
    corrAccX = corrAccY = corrAccZ = 0.0;
    corrGyroX = corrGyroY = corrGyroZ = 0.0;
    
    // Initialize smoothed values
    smoothAccX = smoothAccY = smoothAccZ = 0.0;
    smoothGyroX = smoothGyroY = smoothGyroZ = 0.0;
    
    // Initialize standard deviation values
    accStdDevX = accStdDevY = accStdDevZ = 0.0;
    gyroStdDevX = gyroStdDevY = gyroStdDevZ = 0.0;
    
    // Default thresholds - Adjusted to be more realistic
    motionThreshold = 0.2; // Default threshold is 0.2g
    rolloverThreshold = 0.8; // Vehicle tilt more than ~45 degrees
    hardBrakeThreshold = 0.4; // Deceleration of 0.4g or more
    laneChangeThreshold = 0.25; // Lateral acceleration of 0.25g or more
    rapidAccelThreshold = 0.35; // Acceleration of 0.35g or more
    spinThreshold = 45.0; // Angular velocity of 45 degrees/s or more (reduced from 60)
    vibrationThreshold = 0.4; // Vibration amplitude of 0.4g or more
    
    // Initialize state variables
    lastReadTime = 0;
    lastEventTime = 0;
    lastEventType = NO_EVENT;
    windowIndex = 0;
    filterWindowIndex = 0;
    
    // Default filter settings
    filterEnabled = true;
    filterMethod = SIMPLE_AVERAGE;
    filterAlpha = 0.3; // Default alpha for exponential smoothing (0.3 is a good starting point)
    
    // Initialize callbacks
    motionDetectedCallback = nullptr;
    vehicleEventCallback = nullptr;
    
    // Initialize calibration
    calibrated = false;
    axisOrientation = FORWARD_X_UP_Z; // Default orientation
    
    // Initialize identity rotation matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotationMatrix[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // Initialize event windows
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        accXWindow[i] = accYWindow[i] = accZWindow[i] = 0.0;
        gyroXWindow[i] = gyroYWindow[i] = gyroZWindow[i] = 0.0;
    }
    
    // Initialize filter windows
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        accXFilterWindow[i] = accYFilterWindow[i] = accZFilterWindow[i] = 0.0;
        gyroXFilterWindow[i] = gyroYFilterWindow[i] = gyroZFilterWindow[i] = 0.0;
    }
}

bool AccelerometerGyroClass::begin() {
    // Initialize I2C
    Wire.begin();
    
    // Initialize LSM6DS3
    if (imu.begin() != 0) {
        Serial.println("Failed to initialize IMU!");
        return false;
    }
    
    // Configure sensor for higher data rate (104Hz)
    imu.settings.accelSampleRate = 104;  // 104Hz accelerometer
    imu.settings.gyroSampleRate = 104;   // 104Hz gyroscope
    imu.settings.accelRange = 8;         // ±8g range for acceleration
    imu.settings.gyroRange = 2000;       // ±2000°/s range for gyroscope
    
    if (imu.begin() != 0) {
        Serial.println("Failed to set IMU sample rates!");
        return false;
    }
    
    Serial.println("Accelerometer & Gyro initialized successfully");
    Serial.println("Initial calibration of Z axis...");
    
    // Perform initial Z-axis calibration
    calibrateZAxis();
    
    return true;
}

// Getter methods for raw values
float AccelerometerGyroClass::getAccX() {
    return accX;
}

float AccelerometerGyroClass::getAccY() {
    return accY;
}

float AccelerometerGyroClass::getAccZ() {
    return accZ;
}

float AccelerometerGyroClass::getGyroX() {
    return gyroX;
}

float AccelerometerGyroClass::getGyroY() {
    return gyroY;
}

float AccelerometerGyroClass::getGyroZ() {
    return gyroZ;
}

// Getter methods for corrected values
float AccelerometerGyroClass::getCorrectedAccX() {
    return corrAccX;
}

float AccelerometerGyroClass::getCorrectedAccY() {
    return corrAccY;
}

float AccelerometerGyroClass::getCorrectedAccZ() {
    return corrAccZ;
}

float AccelerometerGyroClass::getCorrectedGyroX() {
    return corrGyroX;
}

float AccelerometerGyroClass::getCorrectedGyroY() {
    return corrGyroY;
}

float AccelerometerGyroClass::getCorrectedGyroZ() {
    return corrGyroZ;
}

// Getter methods for smoothed values
float AccelerometerGyroClass::getSmoothedAccX() {
    return smoothAccX;
}

float AccelerometerGyroClass::getSmoothedAccY() {
    return smoothAccY;
}

float AccelerometerGyroClass::getSmoothedAccZ() {
    return smoothAccZ;
}

float AccelerometerGyroClass::getSmoothedGyroX() {
    return smoothGyroX;
}

float AccelerometerGyroClass::getSmoothedGyroY() {
    return smoothGyroY;
}

float AccelerometerGyroClass::getSmoothedGyroZ() {
    return smoothGyroZ;
}

// Filter control
void AccelerometerGyroClass::enableFilter(bool enable) {
    filterEnabled = enable;
}

bool AccelerometerGyroClass::isFilterEnabled() {
    return filterEnabled;
}

void AccelerometerGyroClass::setFilterMethod(FilterMethod method) {
    filterMethod = method;
}

void AccelerometerGyroClass::setFilterAlpha(float alpha) {
    // Alpha should be between 0 and 1
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    
    filterAlpha = alpha;
}

// Calculate moving average for a given window
float AccelerometerGyroClass::calculateMovingAverage(float* window, int size) {
    float sum = 0.0;
    
    // Sum all values in the window
    for (int i = 0; i < size; i++) {
        sum += window[i];
    }
    
    // Return the average
    return sum / size;
}

// Update filter window with new values
void AccelerometerGyroClass::updateFilterWindow() {
    // Add the corrected values to the filter window
    accXFilterWindow[filterWindowIndex] = corrAccX;
    accYFilterWindow[filterWindowIndex] = corrAccY;
    accZFilterWindow[filterWindowIndex] = corrAccZ;
    gyroXFilterWindow[filterWindowIndex] = corrGyroX;
    gyroYFilterWindow[filterWindowIndex] = corrGyroY;
    gyroZFilterWindow[filterWindowIndex] = corrGyroZ;
    
    // Move to next position in circular buffer
    filterWindowIndex = (filterWindowIndex + 1) % FILTER_WINDOW_SIZE;
}

// Calculate standard deviation for a given window
float AccelerometerGyroClass::calculateStandardDeviation(float* window, int size, float average) {
    float variance = 0.0;
    
    for (int i = 0; i < size; i++) {
        float diff = window[i] - average;
        variance += diff * diff;
    }
    
    variance /= size;
    return sqrt(variance);
}

// Calculate weighted moving average for a given window
float AccelerometerGyroClass::calculateWeightedAverage(float* window, int size) {
    float sum = 0.0;
    float weightSum = 0.0;
    
    // More weight to recent values
    for (int i = 0; i < size; i++) {
        int weight = i + 1; // Weights range from 1 to size
        int idx = (filterWindowIndex - i + size) % size;  // Navigate backwards from current index
        sum += window[idx] * weight;
        weightSum += weight;
    }
    
    // Return the weighted average
    return sum / weightSum;
}

// Apply exponential smoothing filter
float AccelerometerGyroClass::applyExponentialSmoothing(float newValue, float prevSmoothed, float alpha) {
    return alpha * newValue + (1 - alpha) * prevSmoothed;
}

// Apply the sliding window filter to the corrected sensor values
void AccelerometerGyroClass::applyFilter() {
    if (!filterEnabled) {
        // If filtering is disabled, just use corrected values directly
        smoothAccX = corrAccX;
        smoothAccY = corrAccY;
        smoothAccZ = corrAccZ;
        smoothGyroX = corrGyroX;
        smoothGyroY = corrGyroY;
        smoothGyroZ = corrGyroZ;
        return;
    }
    
    // Apply filter based on selected method
    switch (filterMethod) {
        case SIMPLE_AVERAGE:
            // Calculate simple moving averages for all axes
            smoothAccX = calculateMovingAverage(accXFilterWindow, FILTER_WINDOW_SIZE);
            smoothAccY = calculateMovingAverage(accYFilterWindow, FILTER_WINDOW_SIZE);
            smoothAccZ = calculateMovingAverage(accZFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroX = calculateMovingAverage(gyroXFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroY = calculateMovingAverage(gyroYFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroZ = calculateMovingAverage(gyroZFilterWindow, FILTER_WINDOW_SIZE);
            break;
            
        case WEIGHTED_AVERAGE:
            // Calculate weighted moving averages for all axes
            smoothAccX = calculateWeightedAverage(accXFilterWindow, FILTER_WINDOW_SIZE);
            smoothAccY = calculateWeightedAverage(accYFilterWindow, FILTER_WINDOW_SIZE);
            smoothAccZ = calculateWeightedAverage(accZFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroX = calculateWeightedAverage(gyroXFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroY = calculateWeightedAverage(gyroYFilterWindow, FILTER_WINDOW_SIZE);
            smoothGyroZ = calculateWeightedAverage(gyroZFilterWindow, FILTER_WINDOW_SIZE);
            break;
            
        case EXP_SMOOTHING:
            // Apply exponential smoothing to all axes
            smoothAccX = applyExponentialSmoothing(corrAccX, smoothAccX, filterAlpha);
            smoothAccY = applyExponentialSmoothing(corrAccY, smoothAccY, filterAlpha);
            smoothAccZ = applyExponentialSmoothing(corrAccZ, smoothAccZ, filterAlpha);
            smoothGyroX = applyExponentialSmoothing(corrGyroX, smoothGyroX, filterAlpha);
            smoothGyroY = applyExponentialSmoothing(corrGyroY, smoothGyroY, filterAlpha);
            smoothGyroZ = applyExponentialSmoothing(corrGyroZ, smoothGyroZ, filterAlpha);
            break;
    }
    
    // Calculate standard deviation for all axes (for anomaly detection)
    accStdDevX = calculateStandardDeviation(accXFilterWindow, FILTER_WINDOW_SIZE, smoothAccX);
    accStdDevY = calculateStandardDeviation(accYFilterWindow, FILTER_WINDOW_SIZE, smoothAccY);
    accStdDevZ = calculateStandardDeviation(accZFilterWindow, FILTER_WINDOW_SIZE, smoothAccZ);
    gyroStdDevX = calculateStandardDeviation(gyroXFilterWindow, FILTER_WINDOW_SIZE, smoothGyroX);
    gyroStdDevY = calculateStandardDeviation(gyroYFilterWindow, FILTER_WINDOW_SIZE, smoothGyroY);
    gyroStdDevZ = calculateStandardDeviation(gyroZFilterWindow, FILTER_WINDOW_SIZE, smoothGyroZ);
}

// Threshold setters
void AccelerometerGyroClass::setThreshold(float threshold) {
    motionThreshold = threshold;
}

void AccelerometerGyroClass::setRolloverThreshold(float threshold) {
    rolloverThreshold = threshold;
}

void AccelerometerGyroClass::setHardBrakeThreshold(float threshold) {
    hardBrakeThreshold = threshold;
}

void AccelerometerGyroClass::setLaneChangeThreshold(float threshold) {
    laneChangeThreshold = threshold;
}

void AccelerometerGyroClass::setRapidAccelThreshold(float threshold) {
    rapidAccelThreshold = threshold;
}

void AccelerometerGyroClass::setSpinThreshold(float threshold) {
    spinThreshold = threshold;
}

void AccelerometerGyroClass::setVibrationThreshold(float threshold) {
    vibrationThreshold = threshold;
}

// Callbacks
void AccelerometerGyroClass::onMotionDetected(void (*callback)(float, float, float, float, float, float)) {
    motionDetectedCallback = callback;
}

void AccelerometerGyroClass::onVehicleEvent(void (*callback)(VehicleEventType, float, float, float, float, float, float)) {
    vehicleEventCallback = callback;
}

// Axis orientation getters/setters
bool AccelerometerGyroClass::isCalibrated() {
    return calibrated;
}

void AccelerometerGyroClass::setAxisOrientation(AxisOrientation orientation) {
    axisOrientation = orientation;
    
    // Update rotation matrix based on predefined orientations
    switch (orientation) {
        case FORWARD_X_UP_Z:
            // Standard orientation (identity matrix)
            rotationMatrix[0][0] = 1.0; rotationMatrix[0][1] = 0.0; rotationMatrix[0][2] = 0.0;
            rotationMatrix[1][0] = 0.0; rotationMatrix[1][1] = 1.0; rotationMatrix[1][2] = 0.0;
            rotationMatrix[2][0] = 0.0; rotationMatrix[2][1] = 0.0; rotationMatrix[2][2] = 1.0;
            break;
            
        case FORWARD_Y_UP_Z:
            // 90° CW rotation around Z
            rotationMatrix[0][0] = 0.0; rotationMatrix[0][1] = -1.0; rotationMatrix[0][2] = 0.0;
            rotationMatrix[1][0] = 1.0; rotationMatrix[1][1] = 0.0; rotationMatrix[1][2] = 0.0;
            rotationMatrix[2][0] = 0.0; rotationMatrix[2][1] = 0.0; rotationMatrix[2][2] = 1.0;
            break;
            
        case FORWARD_NEG_X_UP_Z:
            // 180° rotation around Z
            rotationMatrix[0][0] = -1.0; rotationMatrix[0][1] = 0.0; rotationMatrix[0][2] = 0.0;
            rotationMatrix[1][0] = 0.0; rotationMatrix[1][1] = -1.0; rotationMatrix[1][2] = 0.0;
            rotationMatrix[2][0] = 0.0; rotationMatrix[2][1] = 0.0; rotationMatrix[2][2] = 1.0;
            break;
            
        case FORWARD_NEG_Y_UP_Z:
            // 90° CCW rotation around Z
            rotationMatrix[0][0] = 0.0; rotationMatrix[0][1] = 1.0; rotationMatrix[0][2] = 0.0;
            rotationMatrix[1][0] = -1.0; rotationMatrix[1][1] = 0.0; rotationMatrix[1][2] = 0.0;
            rotationMatrix[2][0] = 0.0; rotationMatrix[2][1] = 0.0; rotationMatrix[2][2] = 1.0;
            break;
            
        case CUSTOM_ORIENTATION:
            // Keep current custom rotation matrix
            break;
    }
    
    calibrated = (orientation != CUSTOM_ORIENTATION);
}

AxisOrientation AccelerometerGyroClass::getAxisOrientation() {
    return axisOrientation;
}

// Calibration methods
void AccelerometerGyroClass::calibrateZAxis() {
    const int samples = 50;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    Serial.println("Calibrating Z axis. Keep device stationary...");
    
    // Collect multiple samples to average out noise
    for (int i = 0; i < samples; i++) {
        sumX += imu.readFloatAccelX();
        sumY += imu.readFloatAccelY();
        sumZ += imu.readFloatAccelZ();
        delay(20);
    }
    
    // Calculate average acceleration vector
    float avgX = sumX / samples;
    float avgY = sumY / samples;
    float avgZ = sumZ / samples;
    
    // Calculate magnitude of gravity vector
    float magnitude = sqrt(avgX*avgX + avgY*avgY + avgZ*avgZ);
    
    // Normalize to get unit vector in direction of gravity
    gravityVector[0] = avgX / magnitude;
    gravityVector[1] = avgY / magnitude;
    gravityVector[2] = avgZ / magnitude;
    
    Serial.println("Z axis calibration complete");
    Serial.print("Gravity vector: [");
    Serial.print(gravityVector[0]);
    Serial.print(", ");
    Serial.print(gravityVector[1]);
    Serial.print(", ");
    Serial.print(gravityVector[2]);
    Serial.println("]");
    
    // For now, we just have Z calibration
    // Set axis orientation to CUSTOM
    axisOrientation = CUSTOM_ORIENTATION;
    
    // If Z is clearly aligned with one axis, set predefined orientation
    if (abs(gravityVector[0]) < 0.2 && abs(gravityVector[1]) < 0.2 && gravityVector[2] > 0.9) {
        // Z is clearly up
        axisOrientation = FORWARD_X_UP_Z;
        Serial.println("Detected standard orientation with Z up");
    }
    
    calibrated = true;
}

void AccelerometerGyroClass::calibrateXYAxes(float speed) {
    // Only proceed if the vehicle is moving forward
    if (speed < 2.0) {
        Serial.println("Speed too low for X-Y calibration. Need speed > 2.0");
        return;
    }
    
    // We'll use the gyroscope to determine if we're going straight
    // If gyro values are near zero, assume we're moving straight
    if (abs(gyroX) > 5.0 || abs(gyroY) > 5.0 || abs(gyroZ) > 5.0) {
        Serial.println("Vehicle turning detected. Cannot calibrate while turning.");
        return;
    }
    
    Serial.println("Calibrating X-Y axes with forward motion...");
    
    const int samples = 20;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    // Collect multiple samples to average out noise
    for (int i = 0; i < samples; i++) {
        sumX += imu.readFloatAccelX();
        sumY += imu.readFloatAccelY();
        sumZ += imu.readFloatAccelZ();
        delay(20);
    }
    
    // Calculate average acceleration vector
    float avgX = sumX / samples;
    float avgY = sumY / samples;
    float avgZ = sumZ / samples;
    
    // Remove gravity component from this vector
    avgX -= gravityVector[0];
    avgY -= gravityVector[1];
    avgZ -= gravityVector[2];
    
    // Calculate magnitude of forward vector
    float magnitude = sqrt(avgX*avgX + avgY*avgY + avgZ*avgZ);
    
    // If magnitude is too small, we might not be accelerating enough
    if (magnitude < 0.05) {
        Serial.println("Acceleration too low for reliable X-Y calibration");
        return;
    }
    
    // Normalize to get unit vector in direction of forward motion
    forwardVector[0] = avgX / magnitude;
    forwardVector[1] = avgY / magnitude;
    forwardVector[2] = avgZ / magnitude;
    
    Serial.println("X-Y axes calibration complete");
    Serial.print("Forward vector: [");
    Serial.print(forwardVector[0]);
    Serial.print(", ");
    Serial.print(forwardVector[1]);
    Serial.print(", ");
    Serial.print(forwardVector[2]);
    Serial.println("]");
    
    // Now construct rotation matrix from these two reference vectors
    // This is a more complex calculation to create a proper 3D rotation matrix
    // based on the identified gravity and forward vectors
    
    // We're trying to align:
    // gravityVector with [0,0,1] (Z-axis)
    // forwardVector with [1,0,0] (X-axis)
    
    // First, compute right vector (Y-axis) using cross product
    float rightVector[3];
    rightVector[0] = forwardVector[1] * gravityVector[2] - forwardVector[2] * gravityVector[1];
    rightVector[1] = forwardVector[2] * gravityVector[0] - forwardVector[0] * gravityVector[2];
    rightVector[2] = forwardVector[0] * gravityVector[1] - forwardVector[1] * gravityVector[0];
    
    // Normalize right vector
    magnitude = sqrt(rightVector[0]*rightVector[0] + rightVector[1]*rightVector[1] + rightVector[2]*rightVector[2]);
    rightVector[0] /= magnitude;
    rightVector[1] /= magnitude;
    rightVector[2] /= magnitude;
    
    // Recalculate forward vector to ensure orthogonality
    forwardVector[0] = gravityVector[1] * rightVector[2] - gravityVector[2] * rightVector[1];
    forwardVector[1] = gravityVector[2] * rightVector[0] - gravityVector[0] * rightVector[2];
    forwardVector[2] = gravityVector[0] * rightVector[1] - gravityVector[1] * rightVector[0];
    
    // Normalize forward vector again
    magnitude = sqrt(forwardVector[0]*forwardVector[0] + forwardVector[1]*forwardVector[1] + forwardVector[2]*forwardVector[2]);
    forwardVector[0] /= magnitude;
    forwardVector[1] /= magnitude;
    forwardVector[2] /= magnitude;
    
    // Now create rotation matrix
    rotationMatrix[0][0] = forwardVector[0]; rotationMatrix[0][1] = rightVector[0]; rotationMatrix[0][2] = gravityVector[0];
    rotationMatrix[1][0] = forwardVector[1]; rotationMatrix[1][1] = rightVector[1]; rotationMatrix[1][2] = gravityVector[1];
    rotationMatrix[2][0] = forwardVector[2]; rotationMatrix[2][1] = rightVector[2]; rotationMatrix[2][2] = gravityVector[2];
    
    // Set orientation to custom
    axisOrientation = CUSTOM_ORIENTATION;
    calibrated = true;
}

// Event detection methods
bool AccelerometerGyroClass::isRollover() {
    // Calculate pitch and roll angles using the provided formulas
    float pitch = atan2(corrAccX, sqrt(corrAccY*corrAccY + corrAccZ*corrAccZ)) * 180.0/PI;
    float roll = atan2(corrAccY, sqrt(corrAccX*corrAccX + corrAccZ*corrAccZ)) * 180.0/PI;
    
    // Get absolute Z-axis value for orientation check
    float absZ = fabs(corrAccZ);
    
    // Count consecutive samples where angles exceed threshold
    static int highAngleCount = 0;
    
    // Check if pitch or roll exceeds 45 degrees (reduced from 50) and Z is not aligned with gravity
    // Also increased Z threshold from 0.7 to 0.75 to make rollover detection even more sensitive
    if ((abs(pitch) > 45.0 || abs(roll) > 45.0) && absZ < 0.75) {
        highAngleCount++;
        
        // Print debug when we're starting to detect high angles
        if (highAngleCount == 1) {
            Serial.print("Potential rollover - Pitch: ");
            Serial.print(pitch);
            Serial.print("°, Roll: ");
            Serial.print(roll);
            Serial.print("°, Z: ");
            Serial.println(corrAccZ);
        }
    } else {
        highAngleCount = 0;
    }
    
    // Need at least 2 consecutive samples (reduced from 3 to make it even more responsive)
    bool isRollingOver = (highAngleCount >= 2);
    
    // Debug info regularly to help understand values
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 500) {  // Twice per second (more frequent)
        lastDebugPrint = millis();
        
        if (abs(pitch) > 25.0 || abs(roll) > 25.0 || absZ < 0.8) {  // Print more often
            Serial.print("Tilt - Pitch: ");
            Serial.print(pitch);
            Serial.print("°, Roll: ");
            Serial.print(roll);
            Serial.print("°, Z: ");
            Serial.print(corrAccZ);
            Serial.print("g, Count: ");
            Serial.println(highAngleCount);
        }
    }
    
    return isRollingOver;
}

bool AccelerometerGyroClass::isHardBrake() {
    // Hard braking is detected by sustained strong deceleration in forward direction
    // Using threshold of -0.25g for at least 0.5 seconds (5 samples at 10ms sampling)
    
    // Get forward acceleration component 
    float forwardAcc = corrAccX;
    
    // Count consecutive samples with significant deceleration
    static int consecutiveCount = 0;
    
    // Check if forward acceleration is strongly negative (hard braking)
    if (forwardAcc < -0.25) {
        consecutiveCount++;
    } else {
        consecutiveCount = 0;
    }
    
    // Need at least 5 consecutive samples (approximately 0.5s with 10ms sampling)
    bool isHardBraking = (consecutiveCount >= 5);
    
    // Print debug info occasionally
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 1000) {  // Once per second
        lastDebugPrint = millis();
        
        if (forwardAcc < -0.1) {  // Only print if there's some braking
            Serial.print("Hard brake detection - Forward acc: ");
            Serial.print(forwardAcc);
            Serial.print("g, Threshold: -0.25g, Consecutive count: ");
            Serial.println(consecutiveCount);
        }
    }
    
    return isHardBraking;
}

bool AccelerometerGyroClass::isLaneChange() {
    // Lane change is detected by lateral (side-to-side) acceleration
    // or significant rotation around the vertical axis (yaw rate)
    
    // Get lateral acceleration component and yaw rate
    float lateralAcc = corrAccY;
    float yawRate = corrGyroZ;
    
    // Check if either condition is met - increased thresholds to reduce sensitivity
    bool highLateralForce = (fabs(lateralAcc) > 0.4);   // Increased from 0.25 to 0.4
    bool highYawRate = (fabs(yawRate) > 70.0);         // Increased from 50.0 to 70.0
    
    // Start lane change detection
    static unsigned long laneChangeStartTime = 0;
    static unsigned long lastLaneChangeTime = 0;
    static bool inLaneChange = false;
    static int signChangeCount = 0;
    static float lastSignLateral = 0; // Track last direction
    
    // Current time for event tracking
    unsigned long currentTime = millis();
    
    // If we detect high lateral force or yaw rate and not in lane change, start tracking
    if ((highLateralForce || highYawRate) && !inLaneChange) {
        inLaneChange = true;
        laneChangeStartTime = currentTime;
        signChangeCount = 0;
        lastSignLateral = (lateralAcc >= 0) ? 1 : -1;
    }
    
    // Track sign changes in lateral acceleration (typical in lane change)
    if (inLaneChange) {
        float currentSign = (lateralAcc >= 0) ? 1 : -1;
        if (currentSign != lastSignLateral && fabs(lateralAcc) > 0.2) {  // Increased from 0.1 to 0.2
            signChangeCount++;
            lastSignLateral = currentSign;
        }
    }
    
    // If we're in a lane change event, track timing and check for completion
    bool laneChangeDetected = false;
    
    if (inLaneChange) {
        // Lane change typically has a sign change (one direction, then other)
        // and completes within about 1-2 seconds
        unsigned long eventDuration = currentTime - laneChangeStartTime;
        
        // Make detection more strict - require more definitive sign change
        if (eventDuration > 300 && eventDuration < 1500 && 
            (signChangeCount >= 1 && fabs(lateralAcc) < 0.1)) {
            // Lane change is complete, reset state
            inLaneChange = false;
            lastLaneChangeTime = currentTime;
            laneChangeDetected = true;
        } else if (eventDuration > 2000) {
            // Lane change timeout - likely not a lane change but a turn
            inLaneChange = false;
        }
    }
    
    // Print debug info
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 1000) {  // Once per second
        lastDebugPrint = millis();
        
        if (fabs(lateralAcc) > 0.2 || fabs(yawRate) > 30.0) {  // Print for significant movement only
            Serial.print("Lane change detection - Lateral acc: ");
            Serial.print(lateralAcc);
            Serial.print("g, Yaw rate: ");
            Serial.print(yawRate);
            Serial.print("°/s, In event: ");
            Serial.print(inLaneChange ? "Yes" : "No");
            Serial.print(", Sign changes: ");
            Serial.println(signChangeCount);
        }
    }
    
    return laneChangeDetected;
}

bool AccelerometerGyroClass::isRapidAcceleration() {
    // Rapid acceleration is detected by sustained acceleration in forward direction
    // Increased threshold to make it less sensitive
    
    // Get forward acceleration component
    float forwardAcc = corrAccX;
    
    // Track consecutive high acceleration samples
    static int consecutiveCount = 0;
    
    // Check if forward acceleration exceeds threshold - increased from 0.25 to 0.4
    if (forwardAcc > 0.4) {
        consecutiveCount++;
    } else {
        consecutiveCount = 0;
    }
    
    // Need at least 4 consecutive samples (increased from 3 to make less sensitive)
    bool isRapidAccelerating = (consecutiveCount >= 4);
    
    // Print debug info occasionally
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 1000) {  // Once per second
        lastDebugPrint = millis();
        
        if (forwardAcc > 0.2) {  // Only print if there's more significant acceleration
            Serial.print("Rapid acceleration - Forward acc: ");
            Serial.print(forwardAcc);
            Serial.print("g, Threshold: 0.4g, Consecutive count: ");
            Serial.println(consecutiveCount);
        }
    }
    
    return isRapidAccelerating;
}

bool AccelerometerGyroClass::isSpinning() {
    // Spinning is detected by high angular velocity around the Z axis
    // Increased threshold to make detection less sensitive
    
    // Get Z-axis rotation rate
    float zRotationRate = corrGyroZ;
    
    // Track consecutive high rotation samples
    static int consecutiveCount = 0;
    
    // Check if rotation rate exceeds threshold (now 120 deg/s - increased from 100 deg/s)
    if (fabs(zRotationRate) > 120.0) {
        consecutiveCount++;
        
        // Print debug when we start detecting high rotation
        if (consecutiveCount == 1) {
            Serial.print("Potential spin detected - Yaw rate: ");
            Serial.print(zRotationRate);
            Serial.println("°/s");
        }
    } else {
        consecutiveCount = 0;
    }
    
    // Increased required consecutive samples from 3 to 4 to make it less sensitive
    bool isSpinning = (consecutiveCount >= 4);
    
    // Print debug info more frequently
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 500) {  // Twice per second
        lastDebugPrint = millis();
        
        if (fabs(zRotationRate) > 60.0) {  // Increased threshold for debug printing from 30 to 60
            Serial.print("Rotation - Yaw rate: ");
            Serial.print(zRotationRate);
            Serial.print("°/s, Threshold: 120°/s, Count: ");
            Serial.println(consecutiveCount);
        }
    }
    
    return isSpinning;
}

bool AccelerometerGyroClass::isHighVibration() {
    // High vibration is detected by calculating RMS value of acceleration
    // Threshold: RMS > 1.2g for sustained period
    
    // Calculate RMS of acceleration
    float rms = sqrt((corrAccX*corrAccX + corrAccY*corrAccY + corrAccZ*corrAccZ)/3.0);
    
    // Subtract gravity component properly
    // For a stationary device, RMS should be close to 1g due to gravity 
    float normalizedRms = 0;
    if (rms > 1.0) {
        normalizedRms = rms - 1.0; // Subtract gravity (increased from 0.9 to 1.0)
    }
    
    // Track consecutive high vibration samples
    static int consecutiveCount = 0;
    
    // Use much higher threshold for vibration (increased from 0.3 to 0.8)
    if (normalizedRms > 0.8) {  // Significantly increased threshold to avoid false positives
        consecutiveCount++;
    } else {
        // Reset count quickly to avoid false positives
        consecutiveCount = 0;
    }
    
    // Need at least 20 samples showing high vibration (doubled from 10)
    bool isHighVibration = (consecutiveCount >= 20);
    
    // Only print debug info when vibration is high enough to be notable
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 2000) {  // Less frequent (every 2 seconds)
        lastDebugPrint = millis();
        
        if (normalizedRms > 0.4) {  // Only print for significant vibrations
            Serial.print("Vib Debug - RMS: ");
            Serial.print(rms);
            Serial.print("g, Norm: ");
            Serial.print(normalizedRms);
            Serial.print("g, Count: ");
            Serial.println(consecutiveCount);
        }
    }
    
    return isHighVibration;
}

VehicleEventType AccelerometerGyroClass::detectVehicleEvent() {
    // Check for each event type
    // Multiple events can happen simultaneously, but we return only one based on priority
    
    // Check if enough time has passed since last event
    unsigned long currentTime = millis();
    
    // Check regular interval for events
    if ((currentTime - lastEventTime) < MIN_EVENT_INTERVAL) {
        return NO_EVENT;
    }
    
    // Check all possible events and update SensorManager for each detected event
    
    // Rollover is the highest priority - check it first
    if (isRollover()) {
        // Process rollover event
        lastEventType = ROLLOVER_EVENT;
        lastEventTime = currentTime;
        
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(ROLLOVER_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        
        return ROLLOVER_EVENT;  // Return immediately for critical events
    }
    
    // For other events, we'll check all of them and send callbacks for each detected event
    // But return only the highest priority one
    
    bool hardBrakeDetected = isHardBrake();
    bool spinDetected = isSpinning();
    bool rapidAccelDetected = isRapidAcceleration();
    bool laneChangeDetected = isLaneChange();
    bool highVibrationDetected = isHighVibration();
    
    // Process each detected event
    VehicleEventType highestPriorityEvent = NO_EVENT;
    
    if (hardBrakeDetected) {
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(HARD_BRAKE_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        highestPriorityEvent = HARD_BRAKE_EVENT;
    }
    
    if (spinDetected) {
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(SPIN_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        if (highestPriorityEvent == NO_EVENT) {
            highestPriorityEvent = SPIN_EVENT;
        }
    }
    
    if (rapidAccelDetected) {
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(RAPID_ACCEL_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        if (highestPriorityEvent == NO_EVENT) {
            highestPriorityEvent = RAPID_ACCEL_EVENT;
        }
    }
    
    if (laneChangeDetected) {
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(LANE_CHANGE_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        if (highestPriorityEvent == NO_EVENT) {
            highestPriorityEvent = LANE_CHANGE_EVENT;
        }
    }
    
    if (highVibrationDetected) {
        // Call vehicle event callback if registered
        if (vehicleEventCallback != nullptr) {
            vehicleEventCallback(HIGH_VIBRATION_EVENT, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        if (highestPriorityEvent == NO_EVENT) {
            highestPriorityEvent = HIGH_VIBRATION_EVENT;
        }
    }
    
    // Update state if any event was detected
    if (highestPriorityEvent != NO_EVENT) {
        lastEventType = highestPriorityEvent;
        lastEventTime = currentTime;
    }
    
    return highestPriorityEvent;
}

// Apply calibration to raw values
void AccelerometerGyroClass::applyCalibration() {
    if (!calibrated) {
        // If not calibrated, just copy raw values
        corrAccX = accX;
        corrAccY = accY;
        corrAccZ = accZ;
        corrGyroX = gyroX;
        corrGyroY = gyroY;
        corrGyroZ = gyroZ;
        return;
    }
    
    // Apply rotation matrix to accelerometer values
    corrAccX = rotationMatrix[0][0] * accX + rotationMatrix[0][1] * accY + rotationMatrix[0][2] * accZ;
    corrAccY = rotationMatrix[1][0] * accX + rotationMatrix[1][1] * accY + rotationMatrix[1][2] * accZ;
    corrAccZ = rotationMatrix[2][0] * accX + rotationMatrix[2][1] * accY + rotationMatrix[2][2] * accZ;
    
    // Apply rotation matrix to gyroscope values
    corrGyroX = rotationMatrix[0][0] * gyroX + rotationMatrix[0][1] * gyroY + rotationMatrix[0][2] * gyroZ;
    corrGyroY = rotationMatrix[1][0] * gyroX + rotationMatrix[1][1] * gyroY + rotationMatrix[1][2] * gyroZ;
    corrGyroZ = rotationMatrix[2][0] * gyroX + rotationMatrix[2][1] * gyroY + rotationMatrix[2][2] * gyroZ;
}

// Update event window with new values
void AccelerometerGyroClass::updateEventWindow() {
    // Shift values in the window
    accXWindow[windowIndex] = corrAccX;
    accYWindow[windowIndex] = corrAccY;
    accZWindow[windowIndex] = corrAccZ;
    gyroXWindow[windowIndex] = corrGyroX;
    gyroYWindow[windowIndex] = corrGyroY;
    gyroZWindow[windowIndex] = corrGyroZ;
    
    // Update window index
    windowIndex = (windowIndex + 1) % EVENT_WINDOW_SIZE;
}

// Set the READ_INTERVAL to 20ms for ~50Hz sampling rate (50 readings per second)
// This provides much more responsive event detection
void AccelerometerGyroClass::update() {
    unsigned long currentTime = millis();
    
    // Read sensor values at regular intervals (now 50Hz instead of 20Hz)
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Save previous accelerometer values
        lastAccX = accX;
        lastAccY = accY;
        lastAccZ = accZ;
        lastGyroX = gyroX;
        lastGyroY = gyroY;
        lastGyroZ = gyroZ;
        
        // Read new values
        accX = imu.readFloatAccelX();
        accY = imu.readFloatAccelY();
        accZ = imu.readFloatAccelZ();
        gyroX = imu.readFloatGyroX();
        gyroY = imu.readFloatGyroY();
        gyroZ = imu.readFloatGyroZ();
        
        // Apply calibration to convert to vehicle-frame coordinates
        applyCalibration();
        
        // Update sliding window for filtering
        updateFilterWindow();
        
        // Apply moving average filter
        applyFilter();
        
        // Update sliding window of values for event detection
        // NOTE: Here you can decide to use either corrected or smoothed values
        // for event detection - using corrected values for now for responsiveness
        updateEventWindow();
        
        // Check for vehicle events
        VehicleEventType eventType = detectVehicleEvent();
        
        // Note: detectVehicleEvent now directly calls the callbacks for all detected events
        // No additional callback is needed here
        
        // Check if motion exceeds threshold (for basic motion detection)
        float deltaX = abs(accX - lastAccX);
        float deltaY = abs(accY - lastAccY);
        float deltaZ = abs(accZ - lastAccZ);
        
        if (deltaX > motionThreshold || deltaY > motionThreshold || deltaZ > motionThreshold) {
            // Call motion callback if registered
            if (motionDetectedCallback != nullptr) {
                motionDetectedCallback(accX, accY, accZ, gyroX, gyroY, gyroZ);
            }
        }
    }
}

// Add after other methods
void AccelerometerGyroClass::processVehicleEvent(VehicleEventType eventType, float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    // Get event name for display
    const char* eventName = "UNKNOWN";
    
    // Map event type to string name
    switch(eventType) {
        case ROLLOVER_EVENT:
            eventName = "ROLLOVER";
            break;
        case HARD_BRAKE_EVENT:
            eventName = "HARD_BRAKE";
            break;
        case LANE_CHANGE_EVENT:
            eventName = "LANE_CHANGE";
            break;
        case RAPID_ACCEL_EVENT:
            eventName = "RAPID_ACCEL";
            break;
        case SPIN_EVENT:
            eventName = "SPIN";
            break;
        case HIGH_VIBRATION_EVENT:
            eventName = "HIGH_VIBRATION";
            break;
        default:
            eventName = "UNKNOWN";
            break;
    }
    
    // Print simple event notification with sensor data
    Serial.print("EVENT: ");
    Serial.print(eventName);
    Serial.print(" | ACC: X=");
    Serial.print(accX);
    Serial.print("g Y=");
    Serial.print(accY);
    Serial.print("g Z=");
    Serial.print(accZ);
    Serial.print("g | GYRO: X=");
    Serial.print(gyroX);
    Serial.print("°/s Y=");
    Serial.print(gyroY);
    Serial.print("°/s Z=");
    Serial.print(gyroZ);
    Serial.println("°/s");
    
    // Set event in SensorManager
    SensorManager.setEvent(eventName);
}

void AccelerometerGyroClass::defaultMotionHandler(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    // Serial.println("Significant motion detected!");
    // Serial.print("Acceleration: X=");
    // Serial.print(accX);
    // Serial.print("g, Y=");
    // Serial.print(accY);
    // Serial.print("g, Z=");
    // Serial.print(accZ);
    // Serial.println("g");
    
    // Serial.print("Gyroscope: X=");
    // Serial.print(gyroX);
    // Serial.print("°/s, Y=");
    // Serial.print(gyroY);
    // Serial.print("°/s, Z=");
    // Serial.print(gyroZ);
    // Serial.println("°/s");
}

// Check if any axis exceeds the standard deviation threshold
// Useful for detecting unusual movements or sensor anomalies
bool AccelerometerGyroClass::isAnomalyDetected(float stdDevThreshold) {
    // Check if any accelerometer axis exceeds the threshold
    if (accStdDevX > stdDevThreshold || 
        accStdDevY > stdDevThreshold || 
        accStdDevZ > stdDevThreshold) {
        return true;
    }
    
    // Check if any gyroscope axis exceeds the threshold
    if (gyroStdDevX > stdDevThreshold || 
        gyroStdDevY > stdDevThreshold || 
        gyroStdDevZ > stdDevThreshold) {
        return true;
    }
    
    // No anomalies detected
    return false;
} 