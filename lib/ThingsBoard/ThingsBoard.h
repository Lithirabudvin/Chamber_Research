#pragma once

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "SensorManager.h"

struct ThingsBoardConfig {
    String serverUrl;      // e.g., "cloud.thingsnode.cc" or IP address
    int serverPort = 1883; // Default MQTT port (1883 for unencrypted, 8883 for SSL)
    String gatewayToken;
    String wifiSSID;
    String wifiPassword;
    unsigned long sendInterval = 5000;  // 5 seconds for real-time updates
    
    // MQTT options
    bool useSSL = false;
    const char* mqttUsername = nullptr;
    const char* mqttPassword = nullptr;
    
    // NTP Configuration
    const char* ntpServer = "pool.ntp.org";
    long gmtOffset_sec = 0;
    int daylightOffset_sec = 0;
    
    // Persistent MQTT options (NEW)
    bool persistentSession = true;
    String clientId = "";  // If empty, will use MAC-based ID
};

class ThingsBoardClient {
public:
    ThingsBoardClient(const ThingsBoardConfig& config);
    ThingsBoardClient(const char* serverUrl, const char* gatewayToken, 
                      const char* wifiSSID, const char* wifiPassword);
    
    bool begin();
    bool connectWiFi();
    bool connectMQTT();
    bool isWiFiConnected() const { return _wifiConnected; }
    bool isMQTTConnected() { return _mqttConnected && _mqttClient.connected(); }
    bool sendSensorData(const SensorManager& sensorManager);
    bool isSendDue() const;
    String getLastError() const { return _lastError; }
    unsigned long getLastSendTime() const { return _lastSendTime; }
    String getWiFiStatus() const;
    void setSendInterval(unsigned long interval) { _sendInterval = interval; }
    bool isTimeSync() const { return _timeSynced; }
    unsigned long long getEpochMillis();
    
    void loop();  // Call this in your main loop()
    
private:
    ThingsBoardConfig _config;
    unsigned long _sendInterval = 5000;
    
    WiFiClient _wifiClient;
    WiFiClientSecure _wifiClientSecure;
    PubSubClient _mqttClient;
    
    unsigned long _lastSendTime = 0;
    bool _wifiConnected = false;
    bool _mqttConnected = false;
    bool _timeSynced = false;
    bool _devicesConnected = false;  // NEW: Track if devices are already connected
    String _lastError;
    int _failCount = 0;
    String _clientId;  // NEW: Persistent client ID
    
    bool _syncTime();
    bool _checkWiFiConnection();
    bool _checkMQTTConnection();
    bool _sendGatewayTelemetryMQTT(const SensorManager& sensorManager);
    bool _connectDevicesToGateway();
    bool _connectDeviceToGateway(const String& deviceName);
    void _addGatewaySensorData(JsonDocument& doc, const SensorManager& sensorManager);
    void _mqttCallback(char* topic, byte* payload, unsigned int length);
    String _generateClientId();  // NEW: Generate persistent client ID
};