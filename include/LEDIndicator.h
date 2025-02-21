#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <Arduino.h>
#include <PCF8574.h>

class LEDIndicator {
public:
    // Pin definitions as static constexpr members
    static constexpr uint8_t RED_LED_PIN = P4;
    static constexpr uint8_t ORANGE_LED_PIN = P6;
    static constexpr uint8_t BLUE_LED_PIN = P5;
    static constexpr uint16_t DEFAULT_BLINK_INTERVAL = 500;

    // Constructor now has default values from class constants
    LEDIndicator(PCF8574& pcf, 
                 uint8_t redPin = RED_LED_PIN,
                 uint8_t orangePin = ORANGE_LED_PIN,
                 uint8_t bluePin = BLUE_LED_PIN,
                 uint16_t blinkInterval = DEFAULT_BLINK_INTERVAL);
    
    void begin();
    void loop();
    
    // Control methods for each LED
    void setRed(bool on, bool blink = false);
    void setOrange(bool on, bool blink = false);
    void setBlue(bool on, bool blink = false);
    
private:
    PCF8574& _pcf;
    uint8_t _redPin;
    uint8_t _orangePin;
    uint8_t _bluePin;
    uint16_t _blinkInterval;
    
    // LED states
    bool _redOn = false;
    bool _orangeOn = false;
    bool _blueOn = false;
    bool _redBlink = false;
    bool _orangeBlink = false;
    bool _blueBlink = false;
    
    // Timing variables
    unsigned long _lastRedTime = 0;
    unsigned long _lastOrangeTime = 0;
    unsigned long _lastBlueTime = 0;
    
    // Helper method to update LED state
    void updateLED(uint8_t pin, bool on, bool blink, unsigned long& lastTime);
};

#endif 