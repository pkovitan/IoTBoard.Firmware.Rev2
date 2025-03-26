#include "NetworkManager.h"

NetworkManager Network;

NetworkManager::NetworkManager() {
    _isConnected = false;
    _ssid = nullptr;
    _password = nullptr;
}

bool NetworkManager::begin(const char* ssid, const char* password) {
    _ssid = ssid;
    _password = password;

    Serial.println("\n=== WiFi Connection Setup ===");
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(_ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < MAX_CONN_ATTEMPTS) {
        delay(CONN_TIMEOUT);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        _isConnected = true;
        printNetworkStatus();
        return true;
    } else {
        Serial.println("Failed to connect to WiFi!");
        return false;
    }
}

bool NetworkManager::isConnected() {
    _isConnected = (WiFi.status() == WL_CONNECTED);
    return _isConnected;
}

bool NetworkManager::reconnect() {
    Serial.println("\nReconnecting to WiFi network...");
    WiFi.disconnect();
    return begin(_ssid, _password);
}

void NetworkManager::printNetworkStatus() {
    if (isConnected()) {
        Serial.println("\n=== Network Status ===");
        Serial.println("WiFi Connection: ESTABLISHED");
        Serial.print("Connected to SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.println("=====================");
    } else {
        Serial.println("\n=== Network Status ===");
        Serial.println("WiFi Connection: DISCONNECTED");
        Serial.println("=====================");
    }
} 