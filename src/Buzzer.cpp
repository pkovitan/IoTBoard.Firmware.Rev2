#include "Buzzer.h"
#include <Wire.h>

// Create global instance
BuzzerClass Buzzer;

BuzzerClass::BuzzerClass(uint8_t address) : expander(address) {
    isOn = false;
    isPlaying = false;
    startTime = 0;
    duration = 0;
    currentPattern = nullptr;
    patternLength = 0;
    patternIndex = 0;
}

bool BuzzerClass::begin() {
    // Initialize MCP23008
    Wire.begin(21, 22); // SDA, SCL pins
    expander.begin(); // Default pullup = true
    expander.pinMode(BUZZER_PIN, OUTPUT);
    expander.digitalWrite(BUZZER_PIN, LOW);
    
    Serial.println("Buzzer initialized successfully");
    return true;
}

void BuzzerClass::on() {
    isOn = true;
    expander.digitalWrite(BUZZER_PIN, HIGH);
}

void BuzzerClass::off() {
    isOn = false;
    expander.digitalWrite(BUZZER_PIN, LOW);
}

void BuzzerClass::beep(unsigned int duration) {
    isPlaying = true;
    this->duration = duration;
    startTime = millis();
    on();
}

void BuzzerClass::doubleBeep() {
    static const uint8_t pattern[] = {
        1, BEEP_DURATION,
        0, DOUBLE_BEEP_INTERVAL,
        1, BEEP_DURATION,
        0, 0  // End marker
    };
    playPattern(pattern, sizeof(pattern));
}

void BuzzerClass::playPattern(const uint8_t* pattern, uint8_t length) {
    if (!isPlaying) {
        currentPattern = pattern;
        patternLength = length;
        patternIndex = 0;
        isPlaying = true;
        startTime = millis();
    }
}

void BuzzerClass::update() {
    if (!isPlaying) return;
    
    unsigned long currentTime = millis();
    
    // Simple beep mode
    if (currentPattern == nullptr) {
        if (currentTime - startTime >= duration) {
            off();
            isPlaying = false;
        }
    }
    // Pattern playback mode
    else {
        if (currentTime - startTime >= currentPattern[patternIndex + 1]) {
            patternIndex += 2;
            startTime = currentTime;
            
            // Check for end of pattern
            if (patternIndex >= patternLength || currentPattern[patternIndex + 1] == 0) {
                off();
                isPlaying = false;
                currentPattern = nullptr;
                return;
            }
            
            // Set buzzer state according to pattern
            if (currentPattern[patternIndex]) {
                expander.digitalWrite(BUZZER_PIN, HIGH);
                isOn = true;
            } else {
                expander.digitalWrite(BUZZER_PIN, LOW);
                isOn = false;
            }
        }
    }
} 