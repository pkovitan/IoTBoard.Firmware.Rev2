#include "WebFigModule.h"

Config objConfig;
const char* filename = "/config.json";
AsyncWebServer server(80);

// INTRacK Input Parameter
const char* PARAM_WORKINGMODE = "inputWorkingMode";
const char* PARAM_WIFISSID = "inputWifiSSID";
const char* PARAM_WIFIPASSWORD = "inputWifiPassword";
const char* PARAM_MQTTSERVER = "inputMqttServer";
const char* PARAM_MQTTPORT = "inputMqttPort";
const char* PARAM_MQTTUSER = "inputMqttUser";
const char* PARAM_MQTTPASSWORD = "inputMqttPassword";
const char* PARAM_MQTTTOPIC = "inputMqttTopic";
const char* PARAM_MQTTINTERVAL = "inputMqttInterval";
const char* PARAM_HID = "inputHardwareID";

// HTML web page (copy from Legacy-WebFig)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>WebFig for WaTTBoard version 1.0</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    <h2>WEBFiG for INTRacK WATTBoard</h2>
    <hr>
    <br>
    <label for="cars">Choose a car:</label>
      <select name="workingMode" id="workingMode">
        <optgroup label="Select WATTBoard WorkingMode">
          <option value="wifiMode">Wifi Mode</option>
          <option value="lteMode">LTE Mode</option>
        </optgroup>
    </select>
    <br><br>
    WIFI SSID : <input type="text" name="inputWifiSSID" value=%inputWifiSSID%> <br><br>
    WIFI Password : <input type="text" name="inputWifiPassword" value=%inputWifiPassword%> <br><br>
    MQTT Server : <input type="text" name="inputMqttServer" value=%inputMqttServer%> <br><br>    
    MQTT Port : <input type="number" name="inputMqttPort" value=%inputMqttPort%> <br><br>
    MQTT User : <input type="text" name="inputMqttUser" value=%inputMqttUser%> <br><br>    
    MQTT Password : <input type="text" name="inputMqttPassword" value=%inputMqttPassword%> <br><br>    
    MQTT Topic : <input type="text" name="inputMqttTopic" value=%inputMqttTopic%> <br><br>    
    MQTT Interval : <input type="number" name="inputMqttInterval" value=%inputMqttInterval%> <br><br>    
    Hardware ID : <input type="text" name="inputHardwareID" value=%inputHardwareID%> <br><br>   

    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var) {
    if (var == "inputWifiSSID") {
        return objConfig.wifiSSID;
    }
    else if (var == "inputWifiPassword") {
        return objConfig.wifiPassword;
    }
    else if (var == "inputMqttServer") {
        return objConfig.mqttServer;
    }
    else if (var == "inputMqttPort") {
        return String(objConfig.mqttPort);
    }
    else if (var == "inputMqttUser") {
        return objConfig.mqttUser;
    }
    else if (var == "inputMqttPassword") {
        return objConfig.mqttPassword;
    }
    else if (var == "inputMqttTopic") {
        return objConfig.mqttTopic;  
    }
    else if (var == "inputMqttInterval") {
        return String(objConfig.mqttInterval);
    }
    else if (var == "inputHardwareID") {
        return objConfig.hardwareID;
    }
    return String();
}

void loadConfiguration(const char* filename, Config &config) {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Failed to open config file");
        loadDefaultConfig(filename, config);
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("Failed to parse config file");
        loadDefaultConfig(filename, config);
        return;
    }

    strlcpy(config.wifiSSID, doc["wifiSSID"] | ".@Vehicle01", sizeof(config.wifiSSID));
    strlcpy(config.wifiPassword, doc["wifiPassword"] | "intit2021", sizeof(config.wifiPassword));
    strlcpy(config.mqttServer, doc["mqttServer"] | "172.22.1.22", sizeof(config.mqttServer));
    config.mqttPort = doc["mqttPort"] | 1883;
    strlcpy(config.mqttUser, doc["mqttUser"] | "", sizeof(config.mqttUser));
    strlcpy(config.mqttPassword, doc["mqttPassword"] | "", sizeof(config.mqttPassword));
    strlcpy(config.mqttTopic, doc["mqttTopic"] | "INTR.IoT", sizeof(config.mqttTopic));
    config.mqttInterval = doc["mqttInterval"] | 15000;
    strlcpy(config.hardwareID, doc["hardwareID"] | "Vehicle01", sizeof(config.hardwareID));
}

void loadDefaultConfig(const char* filename, Config &config) {
    strcpy(config.wifiSSID, ".@Vehicle01");
    strcpy(config.wifiPassword, "intit2021");
    strcpy(config.mqttServer, "172.22.1.22");
    config.mqttPort = 1883;
    strcpy(config.mqttUser, "");
    strcpy(config.mqttPassword, "");
    strcpy(config.mqttTopic, "INTR.IoT");
    config.mqttInterval = 15000;
    strcpy(config.hardwareID, "Vehicle01");
    
    saveConfiguration(filename, config);
}

void saveConfiguration(const char* filename, Config &config) {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return;
    }

    File file = SPIFFS.open(filename, "w");
    if (!file) {
        Serial.println("Failed to create config file");
        return;
    }

    StaticJsonDocument<512> doc;
    
    doc["wifiSSID"] = config.wifiSSID;
    doc["wifiPassword"] = config.wifiPassword;
    doc["mqttServer"] = config.mqttServer;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttUser"] = config.mqttUser;
    doc["mqttPassword"] = config.mqttPassword;
    doc["mqttTopic"] = config.mqttTopic;
    doc["mqttInterval"] = config.mqttInterval;
    doc["hardwareID"] = config.hardwareID;

    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write config file");
    }

    file.close();
    delay(1000);
    ESP.restart();
}

void initWebFig() {
    // Send web page with input fields to client
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html, processor);
    });

    // Handle form submissions
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (request->hasParam(PARAM_WIFISSID)) {
            strlcpy(objConfig.wifiSSID, 
                   request->getParam(PARAM_WIFISSID)->value().c_str(),
                   sizeof(objConfig.wifiSSID));
        }
        if (request->hasParam(PARAM_WIFIPASSWORD)) {
            strlcpy(objConfig.wifiPassword,
                   request->getParam(PARAM_WIFIPASSWORD)->value().c_str(),
                   sizeof(objConfig.wifiPassword));
        }
        // ... handle other parameters ...
        if (request->hasParam(PARAM_MQTTSERVER)) {
            strlcpy(objConfig.mqttServer,
                   request->getParam(PARAM_MQTTSERVER)->value().c_str(),
                   sizeof(objConfig.mqttServer));
        }
        if (request->hasParam(PARAM_MQTTPORT)) {
            objConfig.mqttPort = request->getParam(PARAM_MQTTPORT)->value().toInt();
        }
        if (request->hasParam(PARAM_MQTTTOPIC)) {
            strlcpy(objConfig.mqttTopic,
                   request->getParam(PARAM_MQTTTOPIC)->value().c_str(),
                   sizeof(objConfig.mqttTopic));
        }
        if (request->hasParam(PARAM_MQTTINTERVAL)) {
            objConfig.mqttInterval = request->getParam(PARAM_MQTTINTERVAL)->value().toInt();
        }
        if (request->hasParam(PARAM_HID)) {
            strlcpy(objConfig.hardwareID,
                   request->getParam(PARAM_HID)->value().c_str(),
                   sizeof(objConfig.hardwareID));
        }
        
        saveConfiguration(filename, objConfig);
        request->send(200, "text/plain", "Configuration saved. Device will restart...");
    });

    server.onNotFound(notFound);
    server.begin();
} 