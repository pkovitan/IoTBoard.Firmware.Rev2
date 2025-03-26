#pragma once

#include <Arduino.h>
#include <WiFi.h>

class NetworkManager {
public:
    NetworkManager();
    
    // Initialize WiFi connection
    bool begin(const char* ssid, const char* password);
    
    // Check if WiFi is connected
    bool isConnected();
    
    // Reconnect to WiFi if disconnected
    bool reconnect();
    
    // Get current connection status
    void printNetworkStatus();

private:
    const char* _ssid;
    const char* _password;
    bool _isConnected;
    
    // Maximum connection attempts
    static const int MAX_CONN_ATTEMPTS = 20;
    
    // Connection timeout in milliseconds
    static const int CONN_TIMEOUT = 500;
};

extern NetworkManager Network; 