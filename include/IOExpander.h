#ifndef IO_EXPANDER_H
#define IO_EXPANDER_H

#include <Arduino.h>
#include "MCP23008.h"

class IOExpander {
public:
    // Pin definitions as static constexpr members
    static constexpr uint8_t EMERGENCY_SW_PIN = 2;  // MCP23008 pin 2
    static constexpr uint8_t DOOR_SW_PIN = 1;       // MCP23008 pin 1
    static constexpr uint8_t GYRO_INT_PIN = 4;      // MCP23008 pin 4
    static constexpr uint8_t ACC_SEN_PIN = 3;       // MCP23008 pin 3
    static constexpr uint8_t BUZZER_PIN = 0;        // MCP23008 pin 0

    // Default I2C pins
    static constexpr uint8_t DEFAULT_SDA_PIN = 21;
    static constexpr uint8_t DEFAULT_SCL_PIN = 22;

    IOExpander(uint8_t sdaPin = DEFAULT_SDA_PIN, uint8_t sclPin = DEFAULT_SCL_PIN);
    
    bool begin();
    void loop();  // Call this in main loop to update states

    // Getters for switch states
    bool isDoorOpen() const { return _doorOpenedFlag; }
    bool isEmergencyOn() const { return _emergencyOnFlag; }
    bool isGyroInterrupt() const;
    bool isAcceleratorActive() const;

    // Buzzer control
    void setBuzzer(bool state);
    void toggleBuzzer();

private:
    MCP23008 _mcp;
    uint8_t _sdaPin;
    uint8_t _sclPin;
    
    // State flags
    bool _doorOpenedFlag;
    bool _emergencyOnFlag;

    // Update states from hardware
    void updateStates();
};

#endif 