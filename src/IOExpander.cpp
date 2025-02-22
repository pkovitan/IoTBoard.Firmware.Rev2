#include "IOExpander.h"

IOExpander::IOExpander(uint8_t sdaPin, uint8_t sclPin)
    : _sdaPin(sdaPin)
    , _sclPin(sclPin)
    , _doorOpenedFlag(false)
    , _emergencyOnFlag(false)
{
}

bool IOExpander::begin() {
    // Initialize MCP23008
    if (!_mcp.begin(_sdaPin, _sclPin)) {
        return false;
    }

    // Configure pin modes
    _mcp.pinMode(DOOR_SW_PIN, INPUT);
    _mcp.pinMode(EMERGENCY_SW_PIN, INPUT);
    _mcp.pinMode(ACC_SEN_PIN, INPUT);
    _mcp.pinMode(GYRO_INT_PIN, INPUT);
    _mcp.pinMode(BUZZER_PIN, OUTPUT);

    // Initial state read
    updateStates();
    
    return true;
}

void IOExpander::loop() {
    updateStates();
}

void IOExpander::updateStates() {
    // Update door state
    _doorOpenedFlag = _mcp.digitalRead(DOOR_SW_PIN);
    
    // Update emergency switch state
    _emergencyOnFlag = _mcp.digitalRead(EMERGENCY_SW_PIN);
}

bool IOExpander::isGyroInterrupt() const {
    return _mcp.digitalRead(GYRO_INT_PIN);
}

bool IOExpander::isAcceleratorActive() const {
    return _mcp.digitalRead(ACC_SEN_PIN);
}

void IOExpander::setBuzzer(bool state) {
    _mcp.digitalWrite(BUZZER_PIN, state);
}

void IOExpander::toggleBuzzer() {
    _mcp.digitalWrite(BUZZER_PIN, !_mcp.digitalRead(BUZZER_PIN));
} 