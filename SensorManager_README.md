# SensorManager

## Overview
SensorManager is a comprehensive module that manages all sensor data collection and payload formatting for the IoTBoard firmware. It reads data from all available sensors and generates payload messages in both JSON and NMEA-0183 formats.

## Features
- Reads data from all sensors (GPS, accelerometer, temperature, etc.)
- Generates standardized payload messages
- Supports two output formats:
  - JSON format for modern applications
  - NMEA-0183 format for compatibility with maritime/navigation systems
- Automatic sensor data collection and payload generation
- Configurable auto-print functionality

## Usage

### Basic Setup
```cpp
#include "SensorManager.h"

void setup() {
  // Initialize SensorManager
  SensorManager.begin();
  
  // Optional: Set identifiers
  SensorManager.setFleet("MyFleet");
  SensorManager.setID("Vehicle123");
  SensorManager.setEvent("NORMAL");
  
  // Optional: Configure auto-print
  SensorManager.enableAutoPrint(true);
  SensorManager.setPrintInterval(1000); // 1 second interval
}

void loop() {
  // Update SensorManager (reads all sensors and updates payloads)
  SensorManager.update();
  
  // Get payload in JSON format
  const char* jsonPayload = SensorManager.getPayload(JSON_FORMAT);
  
  // Or get payload in NMEA-0183 format
  const char* nmeaPayload = SensorManager.getPayload(NMEA_FORMAT);
  
  // Use the payload as needed
  // ...
}
```

### JSON Format
The JSON payload follows this structure:
```json
{
  "fleet": "MyFleet",
  "id": "Vehicle123",
  "seq": 42,
  "time": "2023-01-01T12:34:56.789Z",
  "event": "NORMAL",
  "card-reader": "00000000",
  "vin": 12.5,
  "vbat": 3.7,
  "engine": "ON",
  "speed": 65.5,
  "direction": 270.0,
  "fuel": 75.0,
  "latitude": 13.7563921,
  "longitude": 100.5018549,
  "door": "CLOSE",
  "temp": 25.5,
  "acc-x": 0.01,
  "acc-y": 0.02,
  "acc-z": 1.03,
  "gyr-x": 0.5,
  "gyr-y": 0.5,
  "gyr-z": 0.5,
  "emergency": "FALSE"
}
```

### NMEA-0183 Format
The NMEA-0183 format uses a custom sentence structure:
```
$PSENS,MyFleet,Vehicle123,42,2023-01-01T12:34:56.789Z,NORMAL,00000000,12.5,3.7,ON,65.5,270.0,75.0,13.7563921,100.5018549,CLOSE,25.5,0.01,0.02,1.03,0.5,0.5,0.5,FALSE*XX
```
Where `XX` is the checksum calculated as per NMEA-0183 standard (XOR of all characters between `$` and `*`).

## API Reference

### Public Methods

#### `SensorManagerClass()`
Constructor. Initializes default values.

#### `bool begin()`
Initializes the SensorManager. Returns true if successful.

#### `void update()`
Main update method. Reads all sensor data, updates time, and generates payloads.

#### `void readAllSensors()`
Reads data from all connected sensors.

#### `const char* getPayload(PayloadFormat format)`
Returns the payload string in the specified format (JSON_FORMAT or NMEA_FORMAT).

#### `void printPayload(PayloadFormat format)`
Prints the payload in the specified format to Serial.

#### `void enableAutoPrint(bool enable)`
Enables or disables automatic printing of payloads.

#### `void setPrintInterval(unsigned long interval)`
Sets the interval (in milliseconds) for automatic printing.

#### `void setFleet(const char* fleet)`
Sets the fleet identifier.

#### `void setID(const char* id)`
Sets the vehicle identifier.

#### `void setEvent(const char* event)`
Sets the event type.

## Testing
A separate test file (`SensorManagerTest.cpp`) is provided to demonstrate the SensorManager functionality. It shows how to use both payload formats and alternates between them during runtime. 