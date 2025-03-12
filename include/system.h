#pragma once

#include <Arduino.h>

class SystemClass {
public:
  SystemClass();
  
  // System initialization
  void init();
  
  // Main update loop
  void update();
  
private:
  unsigned long lastUpdateTime;
  
  // Initialize individual components
  void initSerial();
  void initLEDs();
};

extern SystemClass System; 