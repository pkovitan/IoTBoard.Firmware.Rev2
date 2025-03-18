#include "AccelerometerGyro.h"

// Create global instance
AccelerometerGyroClass AccelerometerGyro;

AccelerometerGyroClass::AccelerometerGyroClass() : imu(I2C_MODE, 0x6B) {
    accX = accY = accZ = 0.0;
    gyroX = gyroY = gyroZ = 0.0;
    lastAccX = lastAccY = lastAccZ = 0.0;
    motionThreshold = 0.2; // Default threshold is 0.2g
    lastReadTime = 0;
    motionDetectedCallback = nullptr;
}

bool AccelerometerGyroClass::begin() {
    // Initialize I2C
    Wire.begin();
    
    // Initialize LSM6DS3
    if (imu.begin() != 0) {
        Serial.println("Failed to initialize IMU!");
        return false;
    }
    
    Serial.println("Accelerometer & Gyro initialized successfully");
    return true;
}

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

void AccelerometerGyroClass::setThreshold(float threshold) {
    motionThreshold = threshold;
}

void AccelerometerGyroClass::onMotionDetected(void (*callback)(float, float, float, float, float, float)) {
    motionDetectedCallback = callback;
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
        
        // Read new values
        accX = imu.readFloatAccelX();
        accY = imu.readFloatAccelY();
        accZ = imu.readFloatAccelZ();
        gyroX = imu.readFloatGyroX();
        gyroY = imu.readFloatGyroY();
        gyroZ = imu.readFloatGyroZ();
        
        // Print values
        Serial.print("Acc X: ");
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
        
        // Check if motion exceeds threshold
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
    }
} 