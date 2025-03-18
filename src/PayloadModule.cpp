#include "PayloadModule.h"
#include <time.h>

// Create global instance
PayloadModuleClass PayloadModule;

PayloadModuleClass::PayloadModuleClass() {
    // Initialize default values
    strcpy(fleet, "WillDeleteLater");
    strcpy(id, "Vehicle01");
    seq = 0;
    strncpy(timeString, "2000-01-01T00:00:00.000Z", sizeof(timeString) - 1);
    timeString[sizeof(timeString) - 1] = '\0'; // ตรวจสอบให้แน่ใจว่าสตริงปิดท้ายด้วย null
    strcpy(event, "WillDeleteLater");
    strcpy(cardReader, "00000000");
    vin = 0.0;
    vbat = 0.0;
    strcpy(engine, "OFF");
    speed = 0.0;
    direction = 0.0;
    fuel = 0.0;
    latitude = 0.0;
    longitude = 0.0;
    strcpy(door, "CLOSE");
    temp = 0.0;
    accX = 0.0;
    accY = 0.0;
    accZ = 0.0;
    gyrX = 0.0;
    gyrY = 0.0;
    gyrZ = 0.0;
    strcpy(emergency, "FALSE");
    
    // Initialize payload buffer
    memset(payload, 0, sizeof(payload));
    
    // Initialize auto print settings
    autoPrintEnabled = true;
    printInterval = 1000; // 1 second default
    lastPrintTime = 0;
    
    // Update payload string
    updatePayload();
}

bool PayloadModuleClass::begin() {
    // Nothing special to initialize
    Serial.println("Payload Module initialized successfully");
    return true;
}

void PayloadModuleClass::update() {
    // Update time string
    updateTimeString();
    
    // Update payload string
    updatePayload();
    
    // Check if it's time to print payload
    unsigned long currentTime = millis();
    if (autoPrintEnabled && (currentTime - lastPrintTime >= printInterval)) {
        lastPrintTime = currentTime;
        printPayload();
    }
}

void PayloadModuleClass::setFleet(const char* fleet) {
    strcpy(this->fleet, fleet);
}

void PayloadModuleClass::setID(const char* id) {
    strcpy(this->id, id);
}

void PayloadModuleClass::setEvent(const char* event) {
    strcpy(this->event, event);
}

void PayloadModuleClass::setCardReader(const char* cardID) {
    strcpy(this->cardReader, cardID);
}

void PayloadModuleClass::setVoltageIn(float vin) {
    this->vin = vin;
}

void PayloadModuleClass::setVoltageBattery(float vbat) {
    this->vbat = vbat;
}

void PayloadModuleClass::setEngine(const char* engine) {
    strcpy(this->engine, engine);
}

void PayloadModuleClass::setSpeed(float speed) {
    this->speed = speed;
}

void PayloadModuleClass::setDirection(float direction) {
    this->direction = direction;
}

void PayloadModuleClass::setFuel(float fuel) {
    this->fuel = fuel;
}

void PayloadModuleClass::setLocation(double latitude, double longitude) {
    this->latitude = latitude;
    this->longitude = longitude;
}

void PayloadModuleClass::setDoor(const char* door) {
    strcpy(this->door, door);
}

void PayloadModuleClass::setTemperature(float temp) {
    this->temp = temp;
}

void PayloadModuleClass::setAccelerometer(float x, float y, float z) {
    this->accX = x;
    this->accY = y;
    this->accZ = z;
}

void PayloadModuleClass::setGyroscope(float x, float y, float z) {
    this->gyrX = x;
    this->gyrY = y;
    this->gyrZ = z;
}

void PayloadModuleClass::setEmergency(const char* emergency) {
    strcpy(this->emergency, emergency);
}

const char* PayloadModuleClass::getPayload() {
    return payload;
}

void PayloadModuleClass::printPayload() {
    Serial.println(payload);
}

void PayloadModuleClass::enableAutoPrint(bool enable) {
    autoPrintEnabled = enable;
}

void PayloadModuleClass::setPrintInterval(unsigned long interval) {
    printInterval = interval;
}

void PayloadModuleClass::updatePayload() {
    // Format payload as JSON string
    sprintf(payload, 
        "{\"fleet\":\"%s\",\"id\":\"%s\",\"seq\":%d,\"time\":\"%s\",\"event\":\"%s\",\"card-reader\":\"%s\",\"vin\":%.1f,\"vbat\":%.1f,\"engine\":\"%s\",\"speed\":%.1f,\"direction\":%.1f,\"fuel\":%.1f,\"latitude\":%.7f,\"longitude\":%.7f,\"door\":\"%s\",\"temp\":%.1f,\"acc-x\":%.7f,\"acc-y\":%.7f,\"acc-z\":%.7f,\"gyr-x\":%.7f,\"gyr-y\":%.7f,\"gyr-z\":%.7f,\"emergency\":\"%s\"}",
        fleet, id, seq++, timeString, event, cardReader, vin, vbat, engine, speed, direction, fuel, 
        latitude, longitude, door, temp, accX, accY, accZ, gyrX, gyrY, gyrZ, emergency
    );
}

void PayloadModuleClass::updateTimeString() {
    // Get current time
    time_t now;
    struct tm timeinfo;
    
    // If we have a valid GPS time, we could use that instead
    // For now, just use the system time or create a timestamp
    
    // Format: "2021-12-31T13:00:00.000Z"
    // For demonstration, we'll just use the millis() to update seconds
    
    unsigned long ms = millis() % 1000;
    unsigned long s = (millis() / 1000) % 60;
    unsigned long m = (millis() / 60000) % 60;
    unsigned long h = (millis() / 3600000) % 24;
    
    // This is just a placeholder. In a real application, you would use actual date/time
    sprintf(timeString, "2023-01-01T%02lu:%02lu:%02lu.%03luZ", h, m, s, ms);
} 