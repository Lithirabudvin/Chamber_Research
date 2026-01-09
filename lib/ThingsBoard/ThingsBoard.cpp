#include "ThingsBoard.h"

// Constructor with Config struct
ThingsBoardClient::ThingsBoardClient(const ThingsBoardConfig& config) 
    : _mqttClient(_wifiClient) {  // Initialize with WiFiClient
    _config = config;
    _sendInterval = config.sendInterval;
}

// Original constructor (for backward compatibility)
ThingsBoardClient::ThingsBoardClient(const char* serverUrl, const char* gatewayToken, 
                                     const char* wifiSSID, const char* wifiPassword) 
    : _mqttClient(_wifiClient) {  // Initialize with WiFiClient
    _config.serverUrl = serverUrl;
    _config.gatewayToken = gatewayToken;
    _config.wifiSSID = wifiSSID;
    _config.wifiPassword = wifiPassword;
}

bool ThingsBoardClient::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    
    Serial.printf("[WiFi] SSID: '%s'\n", _config.wifiSSID.c_str());
    
    if (!connectWiFi()) {
        return false;
    }
    
    // Sync time with NTP server
    Serial.println("[NTP] Synchronizing time...");
    if (_syncTime()) {
        Serial.println("[NTP] ✓ Time synchronized");
        unsigned long long epochMs = getEpochMillis();
        time_t epochSec = epochMs / 1000;
        Serial.printf("[NTP] Current time: %s", ctime(&epochSec));
    } else {
        Serial.println("[NTP] ✗ Time sync failed - timestamps will be inaccurate");
    }
    
    // Setup MQTT client based on SSL configuration
    if (_config.useSSL) {
        _wifiClientSecure.setInsecure();
        _mqttClient.setClient(_wifiClientSecure);
    } else {
        _mqttClient.setClient(_wifiClient);
    }
    
    _mqttClient.setServer(_config.serverUrl.c_str(), _config.serverPort);
    _mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->_mqttCallback(topic, payload, length);
    });
    _mqttClient.setBufferSize(4096);
    _mqttClient.setKeepAlive(30);
    
    // Connect to MQTT
    if (!connectMQTT()) {
        Serial.println("[MQTT] Initial connection failed, will retry in loop");
    }
    
    return true;
}

bool ThingsBoardClient::connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        _wifiConnected = true;
        return true;
    }
    
    Serial.println("\n[WiFi] Connecting to: " + _config.wifiSSID);
    
    if (_config.wifiSSID.length() == 0) {
        Serial.println("[WiFi] ERROR: SSID is empty!");
        _lastError = "SSID is empty";
        return false;
    }
    
    // Disconnect first to ensure clean start
    WiFi.disconnect(true);
    delay(100);
    
    WiFi.begin(_config.wifiSSID.c_str(), _config.wifiPassword.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        if (attempts % 10 == 0) {
            Serial.printf("\n[WiFi] Attempt %d/20\n", attempts);
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        _wifiConnected = true;
        Serial.println("\n[WiFi] Connected!");
        Serial.println("  IP Address: " + WiFi.localIP().toString());
        Serial.println("  RSSI: " + String(WiFi.RSSI()) + " dBm");
        return true;
    } else {
        _wifiConnected = false;
        _lastError = "Failed after " + String(attempts) + " attempts";
        Serial.println("\n[WiFi] Connection failed!");
        return false;
    }
}

bool ThingsBoardClient::_syncTime() {
    // Configure time
    configTime(_config.gmtOffset_sec, _config.daylightOffset_sec, _config.ntpServer);
    
    // Wait for time to be set
    int attempts = 0;
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo) && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (attempts < 20) {
        _timeSynced = true;
        return true;
    }
    
    _timeSynced = false;
    return false;
}

unsigned long long ThingsBoardClient::getEpochMillis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long millisecondsSinceEpoch = 
        (unsigned long long)(tv.tv_sec) * 1000 + 
        (unsigned long long)(tv.tv_usec) / 1000;
    return millisecondsSinceEpoch;
}

bool ThingsBoardClient::connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[MQTT] Cannot connect: WiFi not connected");
        return false;
    }
    
    Serial.println("[MQTT] Connecting to broker: " + _config.serverUrl + ":" + String(_config.serverPort));
    
    // Generate client ID
    String clientId = "ESP32Gateway_" + String(random(0xffff), HEX);
    
    // Set last will and testament
    String lastWillTopic = "v1/gateway/disconnect";
    String lastWillMessage = "{\"device\":\"Gateway\"}";
    
    // Clear any existing connection state
    if (_mqttClient.connected()) {
        _mqttClient.disconnect();
        delay(100);
    }
    
    // Connect to MQTT broker
    bool connected = false;
    
    if (_config.mqttUsername && _config.mqttPassword) {
        connected = _mqttClient.connect(
            clientId.c_str(),
            _config.mqttUsername,
            _config.mqttPassword,
            lastWillTopic.c_str(),
            1,
            true,
            lastWillMessage.c_str()
        );
    } else {
        connected = _mqttClient.connect(
            clientId.c_str(),
            _config.gatewayToken.c_str(),
            nullptr,
            lastWillTopic.c_str(),
            1,
            true,
            lastWillMessage.c_str()
        );
    }
    
    if (connected) {
        Serial.println("[MQTT] ✓ Connected successfully!");
        Serial.println("  Client ID: " + clientId);
        
        // Subscribe to attribute updates
        String attributeTopic = "v1/gateway/attributes";
        if (_mqttClient.subscribe(attributeTopic.c_str())) {
            Serial.println("[MQTT] ✓ Subscribed to: " + attributeTopic);
        } else {
            Serial.println("[MQTT] ✗ Failed to subscribe to: " + attributeTopic);
        }
        
        // Subscribe to RPC requests
        String rpcTopic = "v1/gateway/rpc";
        if (_mqttClient.subscribe(rpcTopic.c_str())) {
            Serial.println("[MQTT] ✓ Subscribed to: " + rpcTopic);
        } else {
            Serial.println("[MQTT] ✗ Failed to subscribe to: " + rpcTopic);
        }
        
        // Process initial messages
        _mqttClient.loop();
        delay(100);
        
        // Connect all devices to gateway
        _connectDevicesToGateway();
        
        _mqttConnected = true;
        return true;
    } else {
        _mqttConnected = false;
        _lastError = "MQTT connection failed: " + String(_mqttClient.state());
        Serial.println("[MQTT] ✗ Connection failed! State: " + String(_mqttClient.state()));
        return false;
    }
}

bool ThingsBoardClient::sendSensorData(const SensorManager& sensorManager) {
    // Check connection first
    if (!_checkWiFiConnection()) {
        _lastError = "WiFi not connected";
        return false;
    }
    
    if (!_checkMQTTConnection()) {
        _lastError = "MQTT not connected";
        return false;
    }
    
    // Resync time periodically (every hour)
    static unsigned long lastTimeSync = 0;
    if (millis() - lastTimeSync > 3600000) {  // 1 hour
        if (_syncTime()) {
            Serial.println("[NTP] ✓ Time re-synchronized");
        }
        lastTimeSync = millis();
    }
    
    // Process any pending MQTT messages
    _mqttClient.loop();
    
    // Send data using MQTT Gateway API
    bool success = _sendGatewayTelemetryMQTT(sensorManager);
    
    if (success) {
        _lastSendTime = millis();
        _failCount = 0;
        Serial.println("[Cloud] ✓ Gateway data sent successfully via MQTT");
    } else {
        _failCount++;
        Serial.println("[Cloud] ✗ Failed: " + _lastError);
        
        if (_failCount >= 3) {
            Serial.println("[Cloud] Multiple failures, reconnecting MQTT...");
            _failCount = 0;
            _mqttClient.disconnect();
            _mqttConnected = false;
            delay(500);
            connectMQTT();
        }
    }
    
    return success;
}

bool ThingsBoardClient::isSendDue() const {
    return millis() - _lastSendTime >= _sendInterval;
}

String ThingsBoardClient::getWiFiStatus() const {
    if (WiFi.status() == WL_CONNECTED) {
        return "Connected (" + String(WiFi.RSSI()) + " dBm)";
    }
    return "Disconnected";
}

void ThingsBoardClient::loop() {
    // Process MQTT messages
    _mqttClient.loop();
    
    // Check WiFi periodically
    static unsigned long lastWiFiCheck = 0;
    unsigned long now = millis();
    
    if (now - lastWiFiCheck > 10000) { // Check every 10 seconds
        lastWiFiCheck = now;
        _checkWiFiConnection();
    }
}

bool ThingsBoardClient::_checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        _wifiConnected = false;
        _mqttConnected = false;
        Serial.println("[WiFi] Connection lost, reconnecting...");
        return connectWiFi();
    }
    
    _wifiConnected = true;
    return true;
}

bool ThingsBoardClient::_checkMQTTConnection() {
    if (!_mqttClient.connected()) {
        _mqttConnected = false;
        
        // Check WiFi first
        if (!_checkWiFiConnection()) {
            return false;
        }
        
        // Try to reconnect
        Serial.println("[MQTT] Attempting to reconnect...");
        return connectMQTT();
    }
    
    _mqttConnected = true;
    return true;
}

bool ThingsBoardClient::_sendGatewayTelemetryMQTT(const SensorManager& sensorManager) {
    // Create JSON document for gateway telemetry
    JsonDocument doc;
    
    // Add individual device data in gateway format
    _addGatewaySensorData(doc, sensorManager);
    
    // Check if we have any data to send
    if (doc.size() == 0) {
        Serial.println("[MQTT] No sensor data to send");
        return false;
    }
    
    // Convert to string
    String payload;
    serializeJson(doc, payload);
    
    // Publish to MQTT topic
    const char* topic = "v1/gateway/telemetry";
    
    Serial.println("[MQTT] Publishing to topic: " + String(topic));
    Serial.println("[MQTT] Payload size: " + String(payload.length()) + " bytes");
    
    // Debug: Print first 500 chars of payload
    if (payload.length() > 500) {
        Serial.println("[MQTT] Payload preview: " + payload.substring(0, 500) + "...");
    } else {
        Serial.println("[MQTT] Payload: " + payload);
    }
    
    // Process any pending messages before publishing
    _mqttClient.loop();
    
    // Publish with QoS 1 for reliability
    bool published = _mqttClient.publish(topic, payload.c_str(), true);
    
    // Process messages after publishing
    _mqttClient.loop();
    
    if (published) {
        Serial.println("[MQTT] ✓ Published successfully");
        return true;
    } else {
        _lastError = "MQTT publish failed. State: " + String(_mqttClient.state());
        Serial.println("[MQTT] ✗ Publish failed");
        return false;
    }
}

bool ThingsBoardClient::_connectDevicesToGateway() {
    Serial.println("[Gateway] Connecting devices to gateway via MQTT...");
    
    // List of devices to connect (ADDED SDP810!)
    const char* devices[] = {
        "SCD40(CO2)",
        "SGP41(TVOC)_IN",
        "SGP41(TVOC)_OUT", 
        "SFA30(HCHO)_IN",
        "SFA30(HCHO)_OUT",
        "PMS5003(PM2.5)_IN",
        "PMS5003(PM2.5)_OUT",
        "SHT30(TEMP)_IN",
        "SHT30(TEMP)_OUT",
        "SDP810(AIRFLOW)",  // <- ADDED THIS!
        "Gateway"
    };
    
    bool allConnected = true;
    
    for (const char* deviceName : devices) {
        // Process MQTT messages
        _mqttClient.loop();
        
        if (_connectDeviceToGateway(deviceName)) {
            Serial.printf("[Gateway] ✓ %s connected\n", deviceName);
        } else {
            Serial.printf("[Gateway] ✗ %s connection failed\n", deviceName);
            allConnected = false;
        }
        delay(100); // Increased delay for stability
    }
    
    return allConnected;
}

bool ThingsBoardClient::_connectDeviceToGateway(const String& deviceName) {
    // Create connect message
    JsonDocument doc;
    doc["device"] = deviceName;
    
    String payload;
    serializeJson(doc, payload);
    
    // Publish to connect topic
    const char* topic = "v1/gateway/connect";
    
    // Process messages before publishing
    _mqttClient.loop();
    
    bool published = _mqttClient.publish(topic, payload.c_str(), true);
    
    // Process messages after publishing
    _mqttClient.loop();
    
    if (published) {
        return true;
    }
    
    return false;
}

void ThingsBoardClient::_mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("[MQTT] Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    
    // Print payload
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    
    // Parse JSON if it's an attribute update or RPC request
    if (String(topic) == "v1/gateway/attributes" || String(topic) == "v1/gateway/rpc") {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        
        if (!error) {
            String device = doc["device"] | "Unknown";
            Serial.println("[MQTT] Message for device: " + device);
            
            // Handle attribute updates or RPC requests here
            // You can add custom logic to respond to server commands
        }
    }
}

void ThingsBoardClient::_addGatewaySensorData(JsonDocument& doc, const SensorManager& sensorManager) {
    // Get current Unix timestamp in milliseconds
    unsigned long long currentTs = getEpochMillis();
    
    // If time is not synced, fall back to millis() and warn
    if (!_timeSynced) {
        currentTs = millis();
        static unsigned long lastWarning = 0;
        if (millis() - lastWarning > 60000) {  // Warn every minute
            Serial.println("[Warning] Time not synced - using millis() instead of Unix timestamp");
            lastWarning = millis();
        }
    }
    
    // 1. SCD40 - Device: "SCD40(CO2)"
    if (sensorManager.isSCDActive() && sensorManager.getSCDData().valid) {
        JsonArray deviceArray = doc["SCD40(CO2)"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SCD40::Data& d = sensorManager.getSCDData();
        values["co2"] = d.co2;
        values["temperature"] = d.temperature;
        values["humidity"] = d.humidity;
    }
    
    // 2. SGP41 #1 - Device: "SGP41(TVOC)_IN"
    if (sensorManager.isSGP1Active() && sensorManager.getSGP1Data().valid) {
        JsonArray deviceArray = doc["SGP41(TVOC)_IN"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SGP41::Data& d = sensorManager.getSGP1Data();
        values["voc"] = d.voc;
        values["nox"] = d.nox;
        values["voc_index"] = d.vocIndex;
        values["nox_index"] = d.noxIndex;
    }
    
    // 3. SGP41 #2 - Device: "SGP41(TVOC)_OUT"
    if (sensorManager.isSGP2Active() && sensorManager.getSGP2Data().valid) {
        JsonArray deviceArray = doc["SGP41(TVOC)_OUT"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SGP41::Data& d = sensorManager.getSGP2Data();
        values["voc"] = d.voc;
        values["nox"] = d.nox;
        values["voc_index"] = d.vocIndex;
        values["nox_index"] = d.noxIndex;
    }
    
    // 4. SFA30 #1 - Device: "SFA30(HCHO)_IN"
    if (sensorManager.isSFA1Active() && sensorManager.getSFA1Data().valid) {
        JsonArray deviceArray = doc["SFA30(HCHO)_IN"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SFA30::Data& d = sensorManager.getSFA1Data();
        values["hcho"] = d.formaldehyde;
        values["temperature"] = d.temperature;
        values["humidity"] = d.humidity;
    }
    
    // 5. SFA30 #2 - Device: "SFA30(HCHO)_OUT"
    if (sensorManager.isSFA2Active() && sensorManager.getSFA2Data().valid) {
        JsonArray deviceArray = doc["SFA30(HCHO)_OUT"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SFA30::Data& d = sensorManager.getSFA2Data();
        values["hcho"] = d.formaldehyde;
        values["temperature"] = d.temperature;
        values["humidity"] = d.humidity;
    }
    
    // 6. PMS5003 #1 - Device: "PMS5003(PM2.5)_IN"
    if (sensorManager.isPMS1Active() && sensorManager.getPMS1Data().valid) {
        JsonArray deviceArray = doc["PMS5003(PM2.5)_IN"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const PMS5003::Data& d = sensorManager.getPMS1Data();
        values["pm1_0"] = d.pm10_standard;
        values["pm2_5"] = d.pm25_standard;
        values["pm10"] = d.pm100_standard;
    }
    
    // 7. PMS5003 #2 - Device: "PMS5003(PM2.5)_OUT"
    if (sensorManager.isPMS2Active() && sensorManager.getPMS2Data().valid) {
        JsonArray deviceArray = doc["PMS5003(PM2.5)_OUT"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const PMS5003::Data& d = sensorManager.getPMS2Data();
        values["pm1_0"] = d.pm10_standard;
        values["pm2_5"] = d.pm25_standard;
        values["pm10"] = d.pm100_standard;
    }
    
    // 8. SHT31 #1 - Device: "SHT30(TEMP)_IN"
    if (sensorManager.isSHT1Active() && sensorManager.getSHT1Data().valid) {
        JsonArray deviceArray = doc["SHT30(TEMP)_IN"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SHT31::Data& d = sensorManager.getSHT1Data();
        values["temperature"] = d.temperature;
        values["humidity"] = d.humidity;
        values["heater_enabled"] = d.heaterEnabled;
    }
    
    // 9. SHT31 #2 - Device: "SHT30(TEMP)_OUT"
    if (sensorManager.isSHT2Active() && sensorManager.getSHT2Data().valid) {
        JsonArray deviceArray = doc["SHT30(TEMP)_OUT"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SHT31::Data& d = sensorManager.getSHT2Data();
        values["temperature"] = d.temperature;
        values["humidity"] = d.humidity;
        values["heater_enabled"] = d.heaterEnabled;
    }
    
    // 10. SDP810 - Device: "SDP810(AIRFLOW)" <- ADDED THIS!
    if (sensorManager.isSDPActive() && sensorManager.getSDPData().valid) {
        JsonArray deviceArray = doc["SDP810(AIRFLOW)"].to<JsonArray>();
        JsonObject dataPoint = deviceArray.add<JsonObject>();
        dataPoint["ts"] = currentTs;
        
        JsonObject values = dataPoint["values"].to<JsonObject>();
        const SDP810::Data& d = sensorManager.getSDPData();
        values["differential_pressure"] = d.differential_pressure;
        values["temperature"] = d.temperature;
        values["air_flow"] = d.air_flow;
        values["air_velocity"] = d.air_velocity;
        
        // Add interpretation for easier understanding
        if (abs(d.differential_pressure) < 0.5) {
            values["flow_status"] = "no_flow";
        } else if (d.differential_pressure > 0) {
            values["flow_status"] = "forward";
        } else {
            values["flow_status"] = "reverse";
        }
    }
    
    // 11. Gateway device info
    JsonArray gatewayArray = doc["Gateway"].to<JsonArray>();
    JsonObject gatewayPoint = gatewayArray.add<JsonObject>();
    gatewayPoint["ts"] = currentTs;
    
    JsonObject gatewayValues = gatewayPoint["values"].to<JsonObject>();
    gatewayValues["uptime"] = millis() / 1000;
    gatewayValues["free_heap"] = ESP.getFreeHeap();
    gatewayValues["wifi_rssi"] = WiFi.RSSI();
    gatewayValues["wifi_ssid"] = WiFi.SSID();
    gatewayValues["ip_address"] = WiFi.localIP().toString();
    gatewayValues["time_synced"] = _timeSynced;
    gatewayValues["sdp810_verified"] = sensorManager.getSDPData().valid; // Add SDP810 status
}