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
    // Since calibration is disabled, use a simpler method based on raw Z acceleration
    // In normal position, Z should be close to 1g (when device is flat)
    
    // Debug output to help diagnose the issue
    Serial.print("Z acceleration: ");
    Serial.println(accZ);
    
    // Check if Z axis is significantly different from 1g
    // Only detect rollover when Z is negative (device is upside down)
    return (accZ < -rolloverThreshold);
}

bool AccelerometerGyroClass::isHardBrake() {
    // Hard braking is detected by a sudden deceleration in the forward direction
    
    // Get forward acceleration component 
    float forwardAcc = corrAccX;
    
    // For hard braking, we'll see a strong negative acceleration in forward direction
    // We also check if the previous few readings confirm a sustained deceleration
    
    int countHighDecel = 0;
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        if (accXWindow[i] < -hardBrakeThreshold) {
            countHighDecel++;
        }
    }
    
    // Require at least 2 samples showing high deceleration (reduced from 4)
    return (forwardAcc < -hardBrakeThreshold && countHighDecel >= 2);
}

bool AccelerometerGyroClass::isLaneChange() {
    // Lane change is detected by lateral (side-to-side) acceleration
    
    // Get lateral acceleration component
    float lateralAcc = corrAccY;
    
    // Check for significant lateral acceleration 
    // Add duration check - lateral acceleration should be maintained for a short period
    int countLateralAcc = 0;
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        if (abs(accYWindow[i]) > laneChangeThreshold) {
            countLateralAcc++;
        }
    }
    
    // Require at least 3 samples showing lateral acceleration
    return (abs(lateralAcc) > laneChangeThreshold && countLateralAcc >= 3);
}

bool AccelerometerGyroClass::isRapidAcceleration() {
    // Rapid acceleration is detected by a sudden increase in forward acceleration
    
    // Get forward acceleration component
    float forwardAcc = corrAccX;
    
    // For rapid acceleration, we'll see a strong positive acceleration in forward direction
    // We also check if the previous few readings confirm a sustained acceleration
    
    int countHighAccel = 0;
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        if (accXWindow[i] > rapidAccelThreshold) {
            countHighAccel++;
        }
    }
    
    // Require at least 2 samples showing high acceleration (reduced from 4)
    return (forwardAcc > rapidAccelThreshold && countHighAccel >= 2);
}

bool AccelerometerGyroClass::isSpinning() {
    // Spinning is detected by high angular velocity around the Z axis
    
    // Get Z-axis rotation rate
    float zRotationRate = corrGyroZ;
    
    // Check for significant rotation rate (either direction)
    // Add duration check - spinning should be maintained for a short period
    int countHighRotation = 0;
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        if (abs(gyroZWindow[i]) > spinThreshold) {
            countHighRotation++;
        }
    }
    
    // Require at least 3 samples showing high rotation
    return (abs(zRotationRate) > spinThreshold && countHighRotation >= 3);
}

bool AccelerometerGyroClass::isHighVibration() {
    // High vibration is detected by rapid changes in acceleration values
    
    // Calculate acceleration variance over the window
    float sumX = 0, sumY = 0, sumZ = 0;
    float meanX = 0, meanY = 0, meanZ = 0;
    float varX = 0, varY = 0, varZ = 0;
    
    // Calculate means
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        sumX += accXWindow[i];
        sumY += accYWindow[i];
        sumZ += accZWindow[i];
    }
    
    meanX = sumX / EVENT_WINDOW_SIZE;
    meanY = sumY / EVENT_WINDOW_SIZE;
    meanZ = sumZ / EVENT_WINDOW_SIZE;
    
    // Calculate variances
    for (int i = 0; i < EVENT_WINDOW_SIZE; i++) {
        varX += (accXWindow[i] - meanX) * (accXWindow[i] - meanX);
        varY += (accYWindow[i] - meanY) * (accYWindow[i] - meanY);
        varZ += (accZWindow[i] - meanZ) * (accZWindow[i] - meanZ);
    }
    
    varX /= EVENT_WINDOW_SIZE;
    varY /= EVENT_WINDOW_SIZE;
    varZ /= EVENT_WINDOW_SIZE;
    
    // Calculate total variance (sum of variances in all dimensions)
    float totalVar = varX + varY + varZ;
    
    // Debug output
    Serial.print("Vibration variance: ");
    Serial.println(totalVar);
    
    // Check if variance exceeds threshold and require consecutive high readings
    static int highVibrationCounter = 0;
    
    if (totalVar > vibrationThreshold) {
        highVibrationCounter++;
    } else {
        highVibrationCounter = 0;
    }
    
    // Only report vibration if we have several consecutive high readings
    return (highVibrationCounter >= 5 && totalVar > vibrationThreshold);
}

VehicleEventType AccelerometerGyroClass::detectVehicleEvent() {
    // Check for each event type in order of priority
    
    // Check if enough time has passed since last event
    unsigned long currentTime = millis();
    if ((currentTime - lastEventTime) < MIN_EVENT_INTERVAL) {
        return NO_EVENT;
    }
    
    // Check for most severe events first
    if (isRollover()) {
        lastEventType = ROLLOVER_EVENT;
        lastEventTime = currentTime;
        return ROLLOVER_EVENT;
    }
    
    if (isHardBrake()) {
        lastEventType = HARD_BRAKE_EVENT;
        lastEventTime = currentTime;
        return HARD_BRAKE_EVENT;
    }
    
    if (isSpinning()) {
        lastEventType = SPIN_EVENT;
        lastEventTime = currentTime;
        return SPIN_EVENT;
    }
    
    if (isRapidAcceleration()) {
        lastEventType = RAPID_ACCEL_EVENT;
        lastEventTime = currentTime;
        return RAPID_ACCEL_EVENT;
    }
    
    if (isLaneChange()) {
        lastEventType = LANE_CHANGE_EVENT;
        lastEventTime = currentTime;
        return LANE_CHANGE_EVENT;
    }
    
    if (isHighVibration()) {
        lastEventType = HIGH_VIBRATION_EVENT;
        lastEventTime = currentTime;
        return HIGH_VIBRATION_EVENT;
    }
    
    return NO_EVENT;
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

void AccelerometerGyroClass::update() {
    unsigned long currentTime = millis();
    
    // Read sensor values at regular intervals
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
        
        // Update sliding window of values for event detection
        updateEventWindow();
        
        // Check for vehicle events
        VehicleEventType eventType = detectVehicleEvent();
        
        // Call vehicle event callback if an event was detected
        if (eventType != NO_EVENT && vehicleEventCallback != nullptr) {
            vehicleEventCallback(eventType, corrAccX, corrAccY, corrAccZ, corrGyroX, corrGyroY, corrGyroZ);
        }
        
        // Check if motion exceeds threshold (original simple detection)
        float deltaX = abs(accX - lastAccX);
        float deltaY = abs(accY - lastAccY);
        float deltaZ = abs(accZ - lastAccZ);
        
        if (deltaX > motionThreshold || deltaY > motionThreshold || deltaZ > motionThreshold) {
            // Motion detected
            Serial.println("Motion detected!");
            
            // Call callback if registered
            if (motionDetectedCallback != nullptr) {
                motionDetectedCallback(accX, accY, accZ, gyroX, gyroY, gyroZ);
            }
        }
        
        // Uncomment for debugging
        /*
        Serial.print("Acc Raw X: ");
        Serial.print(accX);
        Serial.print("g, Y: ");
        Serial.print(accY);
        Serial.print("g, Z: ");
        Serial.print(accZ);
        Serial.print("g | Gyro X: ");
        Serial.print(gyroX);
        Serial.print("°/s, Y: ");
        Serial.print(gyroY);
        Serial.print("°/s, Z: ");
        Serial.print(gyroZ);
        Serial.println("°/s");
        
        Serial.print("Acc Corr X: ");
        Serial.print(corrAccX);
        Serial.print("g, Y: ");
        Serial.print(corrAccY);
        Serial.print("g, Z: ");
        Serial.print(corrAccZ);
        Serial.print("g | Gyro X: ");
        Serial.print(corrGyroX);
        Serial.print("°/s, Y: ");
        Serial.print(corrGyroY);
        Serial.print("°/s, Z: ");
        Serial.print(corrGyroZ);
        Serial.println("°/s");
        */
    }
}

// Add after other methods
void AccelerometerGyroClass::processVehicleEvent(VehicleEventType eventType, float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    // Get event name for display
    const char* eventName = "UNKNOWN";
    
    switch (eventType) {
        case ROLLOVER_EVENT:
            eventName = "ROLLOVER";
            Serial.println("===== ROLLOVER EVENT DETAILS =====");
            Serial.println("Check if this is a false alarm!");
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
            Serial.println("===== HIGH VIBRATION EVENT DETAILS =====");
            Serial.println("Check if this is a false alarm!");
            break;
        default:
            eventName = "UNKNOWN";
            break;
    }
    
    Serial.print("VEHICLE EVENT DETECTED: ");
    Serial.println(eventName);
    Serial.print("Acceleration: X=");
    Serial.print(accX);
    Serial.print("g, Y=");
    Serial.print(accY);
    Serial.print("g, Z=");
    Serial.print(accZ);
    Serial.println("g");
    
    Serial.print("Gyroscope: X=");
    Serial.print(gyroX);
    Serial.print("°/s, Y=");
    Serial.print(gyroY);
    Serial.print("°/s, Z=");
    Serial.print(gyroZ);
    Serial.println("°/s");
    
    // Set event in SensorManager
    SensorManager.setEvent(eventName);
}

void AccelerometerGyroClass::defaultMotionHandler(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    Serial.println("Significant motion detected!");
    Serial.print("Acceleration: X=");
    Serial.print(accX);
    Serial.print("g, Y=");
    Serial.print(accY);
    Serial.print("g, Z=");
    Serial.print(accZ);
    Serial.println("g");
    
    Serial.print("Gyroscope: X=");
    Serial.print(gyroX);
    Serial.print("°/s, Y=");
    Serial.print(gyroY);
    Serial.print("°/s, Z=");
    Serial.print(gyroZ);
    Serial.println("°/s");
} 