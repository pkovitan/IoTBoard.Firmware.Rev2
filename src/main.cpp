/**
 * INTrackG v1.0
 * Rewritten by [Your Name]
 * Based on original work by WATT
 * 
 * A vehicle tracking and monitoring system
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "SD.h"
#include "config.h"
#include "system.h"
#include "LEDIndicator.h"
#include "Buzzer.h"
#include "EmergencyButton.h"
#include "DoorSensor.h"
#include "EngineSensor.h"
#include "TemperatureSensor.h"
#include "VoltageMonitor.h"
#include "FuelSensor.h"
#include "CardReader.h"
#include "AccelerometerGyro.h"
#include "GPSModule.h"  // Add GPS Module header
#include "PayloadModule.h" // Add Payload Module header
#include "SDCardLogger.h" // Add SD Card Logger header
#include "NetworkManager.h"
#include "WebFigModule.h"

// Add these constants at the top of the file after includes
#define PAYLOAD_LEN 512
#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 32
#define MAX_SERVER_LEN 32
#define MAX_TOPIC_LEN 32
#define MAX_ID_LEN 32

// Add these default values
// const char* DEFAULT_SSID = ".@Vehicle01";
// const char* DEFAULT_PASSWORD = "intit2021";
// const char* DEFAULT_MQTT_SERVER = "172.22.1.22";
// const int DEFAULT_MQTT_PORT = 1883;
// const char* DEFAULT_MQTT_TOPIC = "INTR.IoT";

WiFiClient intClient;
PubSubClient client(intClient);

// Global variables
char payload[PAYLOAD_LEN];
char payload_fleet[32] = "WillDeleteLater";
char payload_event[32] = "WillDeleteLater";
int payload_seq = 0;
char payload_time[32] = "2021-12-31T13:00:00.000Z";
char payload_card_reader[16] = "00000000";
float payload_vin = 0;
float payload_vbat = 0;
char payload_engine[8] = "OFF";
float payload_speed = 0;
float payload_direction = 0;
float payload_fuel = 0;
double payload_latitude = 0;
double payload_longitude = 0;
char payload_door[8] = "CLOSE";
float payload_temp = 0;
float payload_acc_x = 0, payload_acc_y = 0, payload_acc_z = 0;
float payload_gyr_x = 0, payload_gyr_y = 0, payload_gyr_z = 0;
char payload_emer[8] = "FALSE";

// Control flags
bool emergency_on_flag = false;

// Add these declarations at the top of the file
unsigned long last_time = 0;
unsigned long now_time = 0;
unsigned long last_io_time = 0;
unsigned long last_red_led_time = 0;
unsigned long last_orange_led_time = 0;
unsigned long last_blue_led_time = 0;
bool initial_state = true;
bool acc_on_flag = false;
bool led_blue_on = false;
bool led_blue_blink = false;

// Keep only one set of LED control flags
bool led_red_on = false;
bool led_orange_on = false;
bool led_red_blink = false;
bool led_orange_blink = false;

// SIM7600 definitions
#define SIM_INIT_TIMEOUT 60000
#define SIM_MSG_TIMEOUT 1000
#define SIM_MSG_LEN 256
#define SIM_ACK_LEN 256
HardwareSerial SIM7600_SERIAL(2); // Define Serial2 for SIM7600

// Add these declarations after SIM7600 definitions
char sim_msg[SIM_MSG_LEN];
char sim_ack[SIM_ACK_LEN];

// Add pin definitions for SIM7600
#define RXD2 16
#define TXD2 17
#define S76_PWRKEY 32

// Function prototype
void updatePayloadData();
void led_loop();
void parseLicenseCard();
void read_ex_io();
void getGPS();
void getAnalogSensor();
void getAccGyro();
void writeLog();
void setPayload();
void Expanded_IO_Init();
void Gyro_Init();
void ADC_Init();
void Card_Init();
void initSDCard();
bool SIM7600_init();
void GPS_Init();
bool parseOK();

// Callback function for emergency button press
void emergencyButtonPressed() {
  Serial.println("Emergency Button Pressed!");
}

// Callback function for GPS location change
void locationChanged(double lat, double lon) {
  Serial.print("Location changed to: ");
  Serial.print(lat, 7);
  Serial.print(", ");
  Serial.println(lon, 7);
}

// Callback functions for door sensor
void doorOpened() {
  Serial.println("Door Opened!");
}

void doorClosed() {
  Serial.println("Door Closed!");
}

// Callback functions for engine sensor
void engineStarted() {
  Serial.println("Engine Started!");
  // You can add additional actions here when engine starts
  LedIndicator.setBlue(true); // Turn on blue LED when engine starts
}

void engineStopped() {
  Serial.println("Engine Stopped!");
  // You can add additional actions here when engine stops
  LedIndicator.setBlue(false); // Turn off blue LED when engine stops
}

// Callback function for temperature change
void temperatureChanged(float temperature) {
  Serial.print("Temperature changed to: ");
  Serial.print(temperature);
  Serial.println("°C");
  
  // You can add additional actions here when temperature changes
  // For example, turn on warning if temperature is too high
  if (temperature > 80.0) {
    LedIndicator.setRed(true);
    Buzzer.beep(1000); // Beep for 1 second
  }
}

// Callback function for fuel level change
void fuelLevelChanged(float fuelLevel) {
  Serial.print("Fuel level changed to: ");
  Serial.print(fuelLevel);
  Serial.println("%");
  
  // You can add additional actions here when fuel level changes
  // For example, turn on warning if fuel level is too low
  if (fuelLevel < 10.0) {
    LedIndicator.setOrange(true);
    Buzzer.beep(500); // Beep for 0.5 second
  }
}

// Callback function for card detection
void cardDetected(const char* cardID) {
  Serial.print("Card detected: ");
  Serial.println(cardID);
  
  // You can add additional actions here when a card is detected
  // For example, validate the card ID against a list of authorized cards
  LedIndicator.setRed(true);
  Buzzer.beep(200); // Short beep for card detection
  
  // Turn off green LED after 2 seconds
  delay(2000);
  LedIndicator.setRed(false);
}

// Callback function for motion detection
void motionDetected(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  Serial.println("Significant motion detected!");
  Serial.print("Acceleration: X=");
  Serial.print(accX);
  Serial.print("g, Y=");
  Serial.print(accY);
  Serial.print("g, Z=");
  Serial.print(accZ);
  Serial.println("g");
  
  Serial.print("Gyroscope: X=");
  Serial.print(gyroX);
  Serial.print("°/s, Y=");
  Serial.print(gyroY);
  Serial.print("°/s, Z=");
  Serial.print(gyroZ);
  Serial.println("°/s");
  
  // You can add additional actions here when motion is detected
  // For example, alert if there's a sudden impact or tilt
  if (abs(accX) > 1.5 || abs(accY) > 1.5 || abs(accZ) > 1.5) {
    LedIndicator.setRed(true);
    Buzzer.beep(300); // Beep for impact detection
    delay(1000);
    LedIndicator.setRed(false);
  }
}

void WiFi_init() {
    delay(10);
    
    loadConfiguration(filename, objConfig);
    
    Serial.println("\n=== WiFi Connection Setup ===");
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(objConfig.wifiSSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(objConfig.wifiSSID, objConfig.wifiPassword);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connection Status: SUCCESS");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.println("Web Configuration available at: ");
        Serial.print("http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi Connection Status: FAILED");
        Serial.println("Please use web configuration to set up WiFi");
    }
    Serial.println("============================");
}

void WiFi_reconnect() {
    Serial.println("\nReconnecting to WiFi network...");
    WiFi.disconnect();
    WiFi.begin(objConfig.wifiSSID, objConfig.wifiPassword);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Reconnection: SUCCESS");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi Reconnection: FAILED");
    }
}

void MQTT_init() {
    client.setServer(objConfig.mqttServer, objConfig.mqttPort);
    client.setBufferSize(1024);
}

void MQTT_reconnect() {
    while (!client.connected()) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi_reconnect();
        }
        
        Serial.print("Attempting MQTT connection...");
        
        if (client.connect(objConfig.hardwareID)) {
            Serial.println("connected");
            break;
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

void sendMQTT() {
    #ifdef DEBUG
        Serial.println(payload);
        Serial.println("data sending");
    #endif
    
    client.publish(objConfig.mqttTopic, payload);
    
    // Reset flags after send data
    emergency_on_flag = false;
    strcpy(payload_emer, "FALSE");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    Serial.println("\n=== INTrackG System Initializing ===");
    
    // Initialize components
    Expanded_IO_Init();
    Gyro_Init();
    ADC_Init();
    Card_Init();
    initSDCard();
    
    // Initialize SIM7600
    while(!SIM7600_init()) {
        Serial.println("Retrying SIM7600 initialization...");
    }
    
    // Initialize GPS
    GPS_Init();
    
    // Initialize WiFi and Web Configuration
    WiFi_init();
    initWebFig(); // Initialize web configuration
    
    // Initialize MQTT if WiFi connected
    if (WiFi.status() == WL_CONNECTED) {
        MQTT_init();
        Serial.println("MQTT Initialized");
    }
    
    last_time = millis();
    Serial.println("=== System Initialization Complete ===\n");
}

void loop() {
    led_loop();
    
    // Check WiFi and MQTT connections
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost. Attempting to reconnect...");
        WiFi_reconnect();
        return; // Skip data printing if WiFi not connected
    }
    
    if (!client.connected()) {
        MQTT_reconnect();
        return; // Skip data printing if MQTT not connected
    }
    client.loop();
    
    // Process card reader
    parseLicenseCard();
    
    // Check IO status every second
    if(millis() - last_io_time > 1000) {
        last_io_time = millis();
        read_ex_io();
    }
    
    // Handle engine state changes
    if(initial_state && acc_on_flag) {
        initial_state = false;
        led_blue_on = true;
        led_blue_blink = true;
        #ifdef BUZZER_MOD
            buzzer_on();
            delay(1000);
            buzzer_off();
        #endif   
    }
    
    if(!initial_state && !acc_on_flag) {
        initial_state = true;
        led_blue_on = false;
        led_blue_blink = false;
    }
    
    // Update and send data at specified interval
    now_time = millis();
    if(now_time - last_time > objConfig.mqttInterval) {
        last_time = now_time;
        
        // Update all sensor data
        getGPS();
        getAnalogSensor();
        getAccGyro();
        
        // Print sensor data
        Serial.println("\n=== Sensor Data Update ===");
        Serial.printf("Engine State: %s\n", payload_engine);
        Serial.printf("Door State: %s\n", payload_door);
        Serial.printf("Emergency State: %s\n", payload_emer);
        Serial.printf("Card Reader: %s\n", payload_card_reader);
        Serial.printf("Voltage In: %.2fV, Battery: %.2fV\n", payload_vin, payload_vbat);
        Serial.printf("Fuel Level: %.1f%%\n", payload_fuel);
        Serial.printf("Temperature: %.1f°C\n", payload_temp);
        Serial.printf("GPS: %.7f, %.7f\n", payload_latitude, payload_longitude);
        Serial.printf("Speed: %.1f km/h, Direction: %.1f°\n", payload_speed, payload_direction);
        Serial.printf("Acc: X=%.2f Y=%.2f Z=%.2f\n", payload_acc_x, payload_acc_y, payload_acc_z);
        Serial.printf("Gyro: X=%.2f Y=%.2f Z=%.2f\n", payload_gyr_x, payload_gyr_y, payload_gyr_z);
        Serial.println("=========================\n");
        
        // Update payload and send data
        setPayload();
        writeLog();
        sendMQTT();
        payload_seq++;
    }
}

// Function to update payload data from all sensors
void updatePayloadData() {
  // Set ID and fleet (could be loaded from configuration)
  PayloadModule.setID("Vehicle01");
  PayloadModule.setFleet("WillDeleteLater");
  
  // Set event (could be determined by system state)
  PayloadModule.setEvent("WillDeleteLater");
  
  // Set card reader data
  PayloadModule.setCardReader(CardReader.getCardID());
  
  // Set voltage data
  PayloadModule.setVoltageIn(VoltageMonitor.getVinVoltage());
  PayloadModule.setVoltageBattery(VoltageMonitor.getBatteryVoltage());
  
  // Set engine state
  PayloadModule.setEngine(EngineSensor.isOn() ? "ON" : "OFF");
  
  // Set GPS data
  if (GPSModule.hasValidFix()) {
    PayloadModule.setSpeed(GPSModule.getSpeed());
    PayloadModule.setDirection(GPSModule.getDirection());
    PayloadModule.setLocation(GPSModule.getLatitude(), GPSModule.getLongitude());
  }
  
  // Set fuel level
  PayloadModule.setFuel(FuelSensor.getFuelLevel());
  
  // Set door state
  PayloadModule.setDoor(DoorSensor.isOpen() ? "OPEN" : "CLOSE");
  
  // Set temperature
  PayloadModule.setTemperature(TemperatureSensor.getTemperature());
  
  // Set accelerometer data
  PayloadModule.setAccelerometer(
    AccelerometerGyro.getAccX(),
    AccelerometerGyro.getAccY(),
    AccelerometerGyro.getAccZ()
  );
  
  // Set gyroscope data
  PayloadModule.setGyroscope(
    AccelerometerGyro.getGyroX(),
    AccelerometerGyro.getGyroY(),
    AccelerometerGyro.getGyroZ()
  );
  
  // Set emergency state
  PayloadModule.setEmergency(EmergencyButton.isPressed() ? "TRUE" : "FALSE");
}

void setPayload() {
    memset(payload, 0, sizeof(payload));
    sprintf(payload, "{\"fleet\":\"%s\",\"id\":\"%s\",\"seq\":%d,\"time\":\"%s\",\"event\":\"%s\",\"card-reader\":\"%s\",\"vin\":%.1f,\"vbat\":%.1f,\"engine\":\"%s\",\"speed\":%.1f,\"direction\":%.1f,\"fuel\":%.1f,\"latitude\":%.7f,\"longitude\":%.7f,\"door\":\"%s\",\"temp\":%.1f,\"acc-x\":%.7f,\"acc-y\":%.7f,\"acc-z\":%.7f,\"gyr-x\":%.7f,\"gyr-y\":%.7f,\"gyr-z\":%.7f,\"emergency\":\"%s\"}", 
        payload_fleet, 
        objConfig.hardwareID, 
        payload_seq, 
        payload_time, 
        payload_event, 
        payload_card_reader, 
        payload_vin, 
        payload_vbat, 
        payload_engine, 
        payload_speed, 
        payload_direction, 
        payload_fuel, 
        payload_latitude, 
        payload_longitude, 
        payload_door, 
        payload_temp, 
        payload_acc_x, 
        payload_acc_y, 
        payload_acc_z, 
        payload_gyr_x, 
        payload_gyr_y, 
        payload_gyr_z, 
        payload_emer);
}

void led_loop() {
    // Implementation of led_loop function
}

void parseLicenseCard() {
    // Implementation of parseLicenseCard function
}

void read_ex_io() {
    // Implementation of read_ex_io function
}

void getGPS() {
    // Implementation of getGPS function
}

void getAnalogSensor() {
    // Implementation of getAnalogSensor function
}

void getAccGyro() {
    // Implementation of getAccGyro function
}

void writeLog() {
    // Implementation of writeLog function
}

void Expanded_IO_Init() {
    // Implementation of Expanded_IO_Init function
}

void Gyro_Init() {
    // Implementation of Gyro_Init function
}

void ADC_Init() {
    // Implementation of ADC_Init function
}

void Card_Init() {
    // Implementation of Card_Init function
}

void initSDCard() {
    // Implementation of initSDCard function
}

bool SIM7600_init() {
    SIM7600_SERIAL.begin(115200, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    
    pinMode(S76_PWRKEY, OUTPUT);
    digitalWrite(S76_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(S76_PWRKEY, LOW);
    
    Serial.print("SIM7600 INITING");
    
    led_orange_on = true;
    led_orange_blink = true;
    
    unsigned long init_start_time = millis();
    while((millis() - init_start_time) < SIM_INIT_TIMEOUT) {
        for(int i=0; i<10; i++) {
            led_loop();
            Serial.print(".");
            delay(500);      
        }
        
        SIM7600_SERIAL.println("AT");
        for(int i=0; i<5; i++) {
            led_loop();
            Serial.print(".");
            delay(500);      
        }
        
        if(parseOK()) {
            Serial.println("[DONE]");
            led_orange_blink = false;
            led_loop();
            return true;
        }
    }
    
    Serial.println("[FAIL]");
    return false;
}

void GPS_Init() {
    // Implementation of GPS_Init function
}

bool parseOK() {
    memset(sim_ack, 0, sizeof(sim_ack));
    unsigned long parse_time = millis();
    
    while(SIM7600_SERIAL.available()) {
        SIM7600_SERIAL.readBytes(sim_ack, SIM_ACK_LEN);
        if(strstr(sim_ack, "OK") != NULL) {
            return true;     
        }
        return false;     
    }
    return false;
} 