#pragma once

#include <Arduino.h>

// System configuration
#define FIRMWARE_VERSION "1.0.0"
#define DEBUG true

// Feature flags
#define ENABLE_WIFI true
#define ENABLE_MQTT true
#define ENABLE_SD_LOGGING true
#define ENABLE_BUZZER true

// Pin definitions
// I/O Expander pins
#define PCF8574_RED_PIN P4      // PCF8574 0x20 pin P4
#define PCF8574_ORANGE_PIN P6   // PCF8574 0x20 pin P6
#define PCF8574_BLUE_PIN P5     // PCF8574 0x20 pin P5

// MCP23008 pins (0x22)
#define BUZZER_PIN_MCP 0       // MCP23008 pin 0 for buzzer
#define EMERGENCY_SW_PIN 2     // MCP23008 pin 2
#define DOOR_SW_PIN 1         // MCP23008 pin 1
#define GYRO_INT_PIN 4        // MCP23008 pin 4
#define ENGINE_SEN_PIN 3      // MCP23008 pin 3 for engine detection

// SIM7600 pins
#define RXD2 16
#define TXD2 17
#define S76_PWRKEY 32
#define S76_SLEEP 15
#define S76_PWR_EN 27

// SD Card pin
#define SD_CS_PIN 5

// Card reader pins
#define RXD1 26   
#define TXD1 25

// Buzzer pin definitions
// #define BUZZER_PIN_MCP 0       // MCP23008 pin 0 for buzzer

// Timing configurations
#define DEFAULT_MQTT_INTERVAL 15000  // 15 seconds
#define PCF8574_LED_BLINK_INTERVAL 500  // 0.5 seconds for LED blinking

// Default network configuration
#define DEFAULT_WIFI_SSID "RSEK-CCW8-XCGR"
#define DEFAULT_WIFI_PASSWORD "SmartTMS23"
#define DEFAULT_MQTT_SERVER "172.22.1.22"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_MQTT_TOPIC "INTR.IoT"
#define DEFAULT_HARDWARE_ID "Vehicle01"

// New directory structure
// lib/
//   ├── Config/        # Configuration management
//   ├── Network/       # WiFi and MQTT functionality
//   ├── Sensors/       # Sensor interfaces
//   ├── Storage/       # SD card and logging
//   └── WebConfig/     # Web configuration interface 