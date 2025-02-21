#include "LEDIndicator.h"

LEDIndicator::LEDIndicator(PCF8574& pcf, uint8_t redPin, uint8_t orangePin, uint8_t bluePin, uint16_t blinkInterval)
    : _pcf(pcf)
    , _redPin(redPin)
    , _orangePin(orangePin)
    , _bluePin(bluePin)
    , _blinkInterval(blinkInterval)
{
}

void LEDIndicator::begin() {
    // Initialize all LEDs to OFF (HIGH for active LOW LEDs)
    _pcf.digitalWrite(_redPin, HIGH);
    _pcf.digitalWrite(_orangePin, HIGH);
    _pcf.digitalWrite(_bluePin, HIGH);
}

void LEDIndicator::updateLED(uint8_t pin, bool on, bool blink, unsigned long& lastTime) {
    if (on) {
        if (blink) {
            if ((millis() - lastTime) > _blinkInterval) {
                lastTime = millis();
                _pcf.digitalWrite(pin, !_pcf.digitalRead(pin));
            }
        } else {
            _pcf.digitalWrite(pin, LOW);
        }
    } else {
        _pcf.digitalWrite(pin, HIGH);
    }
}

void LEDIndicator::loop() {
    updateLED(_redPin, _redOn, _redBlink, _lastRedTime);
    updateLED(_orangePin, _orangeOn, _orangeBlink, _lastOrangeTime);
    updateLED(_bluePin, _blueOn, _blueBlink, _lastBlueTime);
}

void LEDIndicator::setRed(bool on, bool blink) {
    _redOn = on;
    _redBlink = blink;
    if (!blink) {
        _pcf.digitalWrite(_redPin, on ? LOW : HIGH);
    }
}

void LEDIndicator::setOrange(bool on, bool blink) {
    _orangeOn = on;
    _orangeBlink = blink;
    if (!blink) {
        _pcf.digitalWrite(_orangePin, on ? LOW : HIGH);
    }
}

void LEDIndicator::setBlue(bool on, bool blink) {
    _blueOn = on;
    _blueBlink = blink;
    if (!blink) {
        _pcf.digitalWrite(_bluePin, on ? LOW : HIGH);
    }
}