#pragma once

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <SPIFFS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <Hash.h>
  #include <FS.h>
#endif
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Configuration structure
struct Config {
    int workingMode;
    char wifiSSID[16];
    char wifiPassword[16];
    char mqttServer[32];
    int mqttPort;
    char mqttUser[16];
    char mqttPassword[16];
    char mqttTopic[64];
    int mqttInterval;
    char hardwareID[16];
};

// Function declarations
void initWebFig();
void loadConfiguration(const char* filename, Config &config);
void saveConfiguration(const char* filename, Config &config);
void loadDefaultConfig(const char* filename, Config &config);

// Make Config object accessible to other modules
extern Config objConfig;
extern const char* filename; 