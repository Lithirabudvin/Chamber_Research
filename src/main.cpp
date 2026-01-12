#include <Arduino.h>
#include "SensorManager.h"
#include "ThingsBoard.h"

// Configuration
const char* WIFI_SSID = "ABC";
const char* WIFI_PASSWORD = "zzom5037";
const char* GATEWAY_TOKEN = "cqb5Xa3H3SkJPJ1NhXtk";

// Global variables
bool sdp810Verified = false;

// Global objects
SensorManager::I2CConfig i2cConfig;
SensorManager::PMSConfig pmsConfig;
SensorManager* sensorManager = nullptr;

// ThingsBoard Client Configuration
ThingsBoardConfig tbConfig;
ThingsBoardClient* thingsBoard = nullptr;

// Timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 30000;

// Utility functions
String getCO2Quality(uint16_t co2) {
    if (co2 < 450) return "Outdoor Fresh";
    else if (co2 < 800) return "Excellent";
    else if (co2 < 1000) return "Good";
    else if (co2 < 1200) return "Moderate";
    else if (co2 < 1500) return "Poor";
    else return "Unhealthy";
}

String getCO2Recommendation(uint16_t co2) {
    if (co2 < 800) return "Maintain ventilation";
    else if (co2 < 1000) return "Increase ventilation";
    else if (co2 < 1200) return "Open windows";
    else return "VENTILATE NOW";
}

String getHCHOQuality(float hcho_ppb) {
    float hcho_ppm = hcho_ppb / 1000.0;
    if (hcho_ppm < 0.08) return "Good";
    else if (hcho_ppm < 0.1) return "Moderate";
    else if (hcho_ppm < 0.2) return "Poor";
    else return "Hazardous";
}

String getVOCQuality(float vocIndex) {
    if (vocIndex < 100) return "Excellent";
    else if (vocIndex < 200) return "Good";
    else if (vocIndex < 300) return "Moderate";
    else if (vocIndex < 400) return "Poor";
    else return "Unhealthy";
}

String getNOxQuality(float noxIndex) {
    if (noxIndex < 1.5) return "Excellent";
    else if (noxIndex < 2.5) return "Good";
    else if (noxIndex < 3.5) return "Moderate";
    else return "Poor";
}

String getTemperatureQuality(float temp) {
    if (temp >= 20.0 && temp <= 24.0) return "Comfortable";
    else if (temp >= 18.0 && temp <= 26.0) return "Acceptable";
    else if (temp < 18.0) return "Too Cold";
    else return "Too Warm";
}

String getHumidityQuality(float humidity) {
    if (humidity >= 40.0 && humidity <= 60.0) return "Ideal";
    else if (humidity >= 30.0 && humidity <= 70.0) return "Acceptable";
    else if (humidity < 30.0) return "Too Dry";
    else return "Too Humid";
}

float calculateDewPoint(float temp, float humidity) {
    if (humidity > 100) humidity = 100;
    if (humidity < 0) humidity = 0;
    
    float a = 17.27;
    float b = 237.7;
    
    float alpha = ((a * temp) / (b + temp)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

float calculateHeatIndex(float temp, float humidity) {
    if (temp < 27.0) return temp;
    
    float c1 = -8.78469475556;
    float c2 = 1.61139411;
    float c3 = 2.33854883889;
    float c4 = -0.14611605;
    float c5 = -0.012308094;
    float c6 = -0.0164248277778;
    float c7 = 0.002211732;
    float c8 = 0.00072546;
    float c9 = -0.000003582;
    
    return c1 + c2*temp + c3*humidity + c4*temp*humidity +
           c5*temp*temp + c6*humidity*humidity + c7*temp*temp*humidity +
           c8*temp*humidity*humidity + c9*temp*temp*humidity*humidity;
}

float calculateAbsoluteHumidity(float temp, float humidity) {
    float saturationVaporPressure = 6.112 * exp((17.67 * temp) / (temp + 243.5));
    float vaporPressure = (humidity / 100.0) * saturationVaporPressure;
    return (216.7 * vaporPressure) / (temp + 273.15);
}

// Display functions
void printSensorStatus() {
    if (!sensorManager) return;
    
    Serial.println("\n════════════════════════════════════════");
    Serial.println("          SENSOR STATUS");
    Serial.println("════════════════════════════════════════");
    Serial.printf("  PMS5003 #1: %s\n", sensorManager->isPMS1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  PMS5003 #2: %s\n", sensorManager->isPMS2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SDP810:     %s\n", sensorManager->isSDPActive() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SCD40:      %s\n", sensorManager->isSCDActive() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SFA30 #1:   %s\n", sensorManager->isSFA1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SFA30 #2:   %s\n", sensorManager->isSFA2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SGP41 #1:   %s", sensorManager->isSGP1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager->isSGP1Active() && sensorManager->getSGP1Data().conditioning) 
        Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.printf("  SGP41 #2:   %s", sensorManager->isSGP2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager->isSGP2Active() && sensorManager->getSGP2Data().conditioning) 
        Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.println("════════════════════════════════════════\n");
}

void printPMSData(const PMS5003::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[PMS5003 #%d] No valid data\n", sensorNum);
        return;
    }
    
    Serial.printf("┌────────────── PM SENSOR #%d ──────────────┐\n", sensorNum);
    Serial.printf("│ PM1.0:  %3d μg/m³                     │\n", data.pm10_standard);
    Serial.printf("│ PM2.5:  %3d μg/m³                     │\n", data.pm25_standard);
    Serial.printf("│ PM10:   %3d μg/m³                     │\n", data.pm100_standard);
    
    // Additional PM data if available
    if (data.pm10_env > 0 || data.pm25_env > 0 || data.pm100_env > 0) {
        Serial.println("├───────────── Environmental ──────────────┤");
        Serial.printf("│ PM1.0 (env): %3d μg/m³              │\n", data.pm10_env);
        Serial.printf("│ PM2.5 (env): %3d μg/m³              │\n", data.pm25_env);
        Serial.printf("│ PM10 (env):  %3d μg/m³              │\n", data.pm100_env);
    }
    
    Serial.println("└──────────────────────────────────────┘");
}

void printSDPData(const SDP810::Data& data) {
    if (!data.valid) {
        Serial.println("[SDP810] No valid data");
        return;
    }
    
    Serial.println("┌────────────── AIR FLOW ───────────────┐");
    Serial.printf("│ Pressure: %+7.2f Pa                 │\n", data.differential_pressure);
    Serial.printf("│ Temp:     %7.2f °C                 │\n", data.temperature);
    Serial.printf("│ Flow:     %7.4f m³/s              │\n", data.air_flow);
    Serial.printf("│ Velocity: %7.2f m/s                │\n", data.air_velocity);
    
    Serial.println("├──────────── Interpretation ───────────┤");
    if (abs(data.differential_pressure) < 1.0) {
        Serial.println("│ Status:   No significant air flow    │");
    } else if (data.differential_pressure > 0) {
        Serial.printf("│ Direction: Forward (+)%16s│\n", "");
    } else {
        Serial.printf("│ Direction: Reverse (-)%16s│\n", "");
    }
    Serial.println("└──────────────────────────────────────┘");
}

void printSCDData(const SCD40::Data& data) {
    if (!data.valid) {
        Serial.println("[SCD40] No valid data yet");
        return;
    }
    
    String quality = getCO2Quality(data.co2);
    String recommendation = getCO2Recommendation(data.co2);
    
    Serial.println("┌────────────── CO₂ SENSOR ──────────────┐");
    Serial.printf("│ CO₂:         %5d ppm               │\n", data.co2);
    Serial.printf("│ Temperature:   %5.1f °C              │\n", data.temperature);
    Serial.printf("│ Humidity:      %5.1f %%               │\n", data.humidity);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Quality:       %-22s│\n", quality.c_str());
    Serial.printf("│ Action:        %-22s│\n", recommendation.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

String getHCHOHealthRisk(float hcho_ppb) {
    if (hcho_ppb < 80) return "✅ Safe";
    else if (hcho_ppb < 100) return "⚠️  Caution";
    else if (hcho_ppb < 200) return "⚠️  Moderate";
    else if (hcho_ppb < 400) return "🔴 Strong irritation";
    else if (hcho_ppb < 800) return "🔴 Severe";
    else if (hcho_ppb < 1500) return "🔴🔴 EXTREMELY DANGEROUS";
    else return "🔴🔴🔴 CRITICAL - EVACUATE";
}

void printSFAData(const SFA30::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[SFA30 #%d] No valid data\n", sensorNum);
        return;
    }
    
    // Don't skip zeros - show warning but still display
    if (data.formaldehyde == 0.0 && data.temperature == 0.0 && data.humidity == 0.0) {
        Serial.printf("[SFA30 #%d] ⚠️  Sensor returning zeros - may be disconnected\n", sensorNum);
        return;
    }
    
    String quality = getHCHOQuality(data.formaldehyde);
    String healthRisk = getHCHOHealthRisk(data.formaldehyde);
    
    Serial.printf("┌────────────── HCHO SENSOR #%d ─────────────┐\n", sensorNum);
    Serial.printf("│ HCHO:      %8.1f ppb              │\n", data.formaldehyde);
    Serial.printf("│            %8.3f ppm              │\n", data.formaldehyde / 1000.0);
    Serial.printf("│ Temperature:   %5.1f °C              │\n", data.temperature);
    Serial.printf("│ Humidity:      %5.1f %%               │\n", data.humidity);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Quality:       %-22s│\n", quality.c_str());
    Serial.printf("│ Health Risk:   %-22s│\n", healthRisk.c_str());
    
    // Show WHO comparison for high readings
    if (data.formaldehyde > 80) {
        float who_limit = 80.0; // ppb
        float ratio = data.formaldehyde / who_limit;
        Serial.printf("│ WHO Limit:     %.1fx EXCEEDED           │\n", ratio);
    }
    
    // Show trend indicator if you want (optional)
    static float lastReading1 = 0;
    static float lastReading2 = 0;
    float* lastReading = (sensorNum == 1) ? &lastReading1 : &lastReading2;
    
    if (*lastReading > 0) {
        float change = data.formaldehyde - *lastReading;
        if (abs(change) > 10) { // Only show if significant change
            if (change > 0) {
                Serial.printf("│ Trend:         ↑ Rising (+%.0f ppb)       │\n", change);
            } else {
                Serial.printf("│ Trend:         ↓ Falling (%.0f ppb)      │\n", change);
            }
        }
    }
    *lastReading = data.formaldehyde;
    
    Serial.println("└──────────────────────────────────────┘");
}

void printSGPData(const SGP41::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[SGP41 #%d] No valid data\n", sensorNum);
        return;
    }
    
    String vocQuality = getVOCQuality(data.vocIndex);
    String noxQuality = getNOxQuality(data.noxIndex);
    
    Serial.printf("┌────────────── VOC/NOx #%d ─────────────┐\n", sensorNum);
    Serial.printf("│ VOC:         %6d (raw)           │\n", data.voc);
    Serial.printf("│ VOC Index:   %6.0f                │\n", data.vocIndex);
    Serial.printf("│ NOx:         %6d (raw)           │\n", data.nox);
    Serial.printf("│ NOx Index:   %6.1f                │\n", data.noxIndex);
    
    if (data.conditioning) {
        Serial.println("├──────────────────────────────────────┤");
        Serial.println("│ Status:      CONDITIONING (10m)      │");
    }
    
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ VOC Quality:   %-22s│\n", vocQuality.c_str());
    Serial.printf("│ NOx Quality:   %-22s│\n", noxQuality.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void printSHTData(const SHT31::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[SHT31 #%d] No valid data\n", sensorNum);
        return;
    }
    
    String tempQuality = getTemperatureQuality(data.temperature);
    String humQuality = getHumidityQuality(data.humidity);
    float dewPoint = calculateDewPoint(data.temperature, data.humidity);
    float heatIndex = calculateHeatIndex(data.temperature, data.humidity);
    float absHumidity = calculateAbsoluteHumidity(data.temperature, data.humidity);
    
    Serial.printf("┌──────────── ENVIRONMENT #%d ───────────┐\n", sensorNum);
    Serial.printf("│ Temperature:   %5.1f °C              │\n", data.temperature);
    Serial.printf("│ Humidity:      %5.1f %%               │\n", data.humidity);
    Serial.printf("│ Dew Point:     %5.1f °C              │\n", dewPoint);
    
    if (data.temperature >= 27.0) {
        Serial.printf("│ Heat Index:    %5.1f °C              │\n", heatIndex);
    }
    
    Serial.printf("│ Abs. Humidity: %5.1f g/m³            │\n", absHumidity);
    

    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Temp Quality:  %-22s│\n", tempQuality.c_str());
    Serial.printf("│ Hum Quality:   %-22s│\n", humQuality.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

// ADD THIS FUNCTION TO VERIFY SDP810
void verifySDP810() {
    if (!sensorManager) return;
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║        VERIFYING SDP810 SENSOR        ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Use the SDP810 sensor from sensorManager
    if (sensorManager->isSDPActive()) {
        Serial.println("[SDP810] Sensor detected at address 0x25");
        
        // Try to read 5 times to verify
        int successfulReads = 0;
        float avgPressure = 0;
        float avgTemp = 0;
        
        for (int i = 0; i < 5; i++) {
            if (sensorManager->readSDP()) {
                const SDP810::Data& data = sensorManager->getSDPData();
                if (data.valid) {
                    successfulReads++;
                    avgPressure += data.differential_pressure;
                    avgTemp += data.temperature;
                    
                    Serial.printf("  Read #%d: Pressure=%+7.2f Pa, Temp=%6.2f°C\n", 
                                 i+1, data.differential_pressure, data.temperature);
                }
            }
            delay(500);
        }
        
        if (successfulReads > 0) {
            avgPressure /= successfulReads;
            avgTemp /= successfulReads;
            
            Serial.println("\n[VERIFICATION RESULTS]");
            Serial.printf("  Successful reads: %d/5\n", successfulReads);
            Serial.printf("  Average pressure: %+7.2f Pa\n", avgPressure);
            Serial.printf("  Average temperature: %6.2f°C\n", avgTemp);
            
            // Check if values are reasonable
            bool tempReasonable = (avgTemp > -40 && avgTemp < 125);
            bool pressureReasonable = (abs(avgPressure) < 1000);
            
            if (tempReasonable && pressureReasonable) {
                Serial.println("  ✓ SDP810 is WORKING CORRECTLY!");
                sdp810Verified = true;
            } else {
                if (!tempReasonable) Serial.println("  ✗ Temperature out of range!");
                if (!pressureReasonable) Serial.println("  ✗ Pressure out of range!");
            }
        } else {
            Serial.println("  ✗ Failed to read any valid data!");
        }
    } else {
        Serial.println("  ✗ SDP810 sensor not detected!");
    }
    
    Serial.println("\n════════════════════════════════════════\n");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n════════════════════════════════════════");
    Serial.println("    ESP32 AIR QUALITY MONITORING");
    Serial.println("    with ThingsBoard Gateway MQTT");
    Serial.println("    and SDP810 Air Flow Sensor");
    Serial.println("════════════════════════════════════════");
    Serial.println("  Version: 2.2 (Dual I2C Bus Support)");
    Serial.println("  ThingsBoard Server: cloud.thingsnode.cc");
    Serial.println("  Gateway Token: " + String(GATEWAY_TOKEN));
    Serial.println("  WiFi: " + String(WIFI_SSID));
    Serial.println("════════════════════════════════════════\n");
    
    // Configure I2C pins for all sensors
    i2cConfig.mainSDA = 21;
    i2cConfig.mainSCL = 22;
    i2cConfig.altSDA = 25;   // For SFA30 #2 and SGP41 #2
    i2cConfig.altSCL = 26;
    
    // PMS config
    pmsConfig.rxPin1 = 16;
    pmsConfig.txPin1 = 17;
    pmsConfig.rxPin2 = 18;
    pmsConfig.txPin2 = 19;
    
    // Create sensor manager with new config
    sensorManager = new SensorManager(i2cConfig, pmsConfig);
    
    // Initialize sensor manager
    Serial.println("\n[System] Initializing 12 sensors with dual I2C bus support...");
    Serial.println("  Main Bus (21/22): SDP810, SCD40, SHT31 #1, SHT31 #2, SFA30 #1, SGP41 #1");
    Serial.println("  Alt Bus (25/26):  SFA30 #2, SGP41 #2");
    
    if (!sensorManager->begin()) {
        Serial.println("[ERROR] Failed to initialize sensors!");
        while (1) {
            Serial.println("System halted. Please check connections and restart.");
            delay(5000);
        }
    }
    
    // VERIFY SDP810 BEFORE CONTINUING
    verifySDP810();
    
    // Only continue if SDP810 is verified or user wants to continue anyway
    if (!sdp810Verified) {
        Serial.println("\n⚠ WARNING: SDP810 verification failed!");
        Serial.println("  Options:");
        Serial.println("  1. Check sensor wiring (SDA=21, SCL=22)");
        Serial.println("  2. Verify sensor is powered (3.3V)");
        Serial.println("  3. Press RESET to try again");
        Serial.println("  4. Continue without SDP810 (data may be invalid)");
        Serial.println("\nContinuing in 5 seconds...");
        delay(5000);
    }
    
    // Configure ThingsBoard
    tbConfig.serverUrl = "cloud.thingsnode.cc";
    tbConfig.serverPort = 1883;
    tbConfig.gatewayToken = GATEWAY_TOKEN;
    tbConfig.wifiSSID = WIFI_SSID;
    tbConfig.wifiPassword = WIFI_PASSWORD;
    tbConfig.sendInterval = 15000;  // 15 seconds is fine for most applications
    tbConfig.useSSL = false;
    tbConfig.gmtOffset_sec = 19800;
    tbConfig.daylightOffset_sec = 0;
    
    // NEW: Add persistent configuration
    tbConfig.persistentSession = true;
    tbConfig.clientId = "";  // Will auto-generate from MAC
    
    thingsBoard = new ThingsBoardClient(tbConfig);
Serial.println("\n[System] Initializing WiFi and MQTT Gateway...");
    if (thingsBoard->begin()) {
        Serial.println("[System] ✓ WiFi connected");
        
        if (thingsBoard->isTimeSync()) {
            Serial.println("[System] ✓ Time synchronized with NTP");
        }
        
        Serial.println("[System] Waiting for MQTT and gateway setup to complete...");
        
        // CRITICAL: Wait for full gateway setup
        unsigned long startWait = millis();
        while (millis() - startWait < 10000) {  // Wait up to 10 seconds
            thingsBoard->loop();  // Process MQTT messages
            delay(100);
            
            // Check if connected
            if (thingsBoard->isMQTTConnected()) {
                Serial.println("[System] ✓ MQTT and gateway ready");
                break;
            }
        }
        
        if (!thingsBoard->isMQTTConnected()) {
            Serial.println("[System] ⚠ MQTT setup taking longer than expected");
            Serial.println("[System] Continuing, will retry in main loop");
        }
    } else {
        Serial.println("[System] ✗ WiFi connection failed!");
        Serial.println("  Continuing in offline mode...");
        Serial.println("  Local monitoring active, Cloud upload disabled.");
    }
    
    delay(3000);
    
    Serial.println("\n[Debug] Checking MQTT connection...");
    Serial.println("  MQTT Connected: " + String(thingsBoard->isMQTTConnected() ? "YES" : "NO"));
    Serial.println("  Time Synced: " + String(thingsBoard->isTimeSync() ? "YES" : "NO"));
    Serial.println("  Gateway Token: " + String(GATEWAY_TOKEN));
    
    // Print initial status
    printSensorStatus();
    
    Serial.println("\n════════════════════════════════════════");
    Serial.println("        STARTING MONITORING");
    Serial.println("════════════════════════════════════════");
    if (sdp810Verified) {
        Serial.println("  SDP810: ✓ VERIFIED AND WORKING");
    } else {
        Serial.println("  SDP810: ⚠ NOT VERIFIED / DISABLED");
    }
    Serial.println("  Main Bus (21/22): SDP810, SCD40, SHT31, SFA30 #1, SGP41 #1");
    Serial.println("  Alt Bus (25/26):  SFA30 #2, SGP41 #2");
    Serial.println("Local display interval:   5 seconds");
    Serial.println("Cloud upload interval:    5 seconds (REAL-TIME)");
    Serial.println("Total sensors:           12");
    Serial.println("Cloud protocol:          MQTT Gateway");
    Serial.println("Timestamp format:        Unix epoch (ms)");
    Serial.println("════════════════════════════════════════\n");
    
    Serial.println("Starting measurements in 3 seconds...");
    delay(3000);
}

void loop() {
    static bool sdpOk = false;
    static unsigned long lastReadTime = 0;
    static unsigned long lastSDP810SimpleRead = 0;
    
    if (!sensorManager) return;
    
    unsigned long now = millis();
    
    // CRITICAL: Call MQTT loop frequently for real-time operation
    if (thingsBoard) {
        thingsBoard->loop();
    }
    
    // Read all sensors every 2 seconds
    if (now - lastReadTime >= 2000) {
        lastReadTime = now;
        
        // Read all sensors
        sensorManager->readPMS1();
        sensorManager->readPMS2();
        
        // Read SDP810 (only if verified)
        if (sensorManager->isSDPActive() && sdp810Verified) {
            sdpOk = sensorManager->readSDP();
            
            // Optional: Add simple verification during operation
            if (now - lastSDP810SimpleRead >= 30000) { // Every 30 seconds
                lastSDP810SimpleRead = now;
                
                if (sdpOk) {
                    const SDP810::Data& data = sensorManager->getSDPData();
                    if (data.temperature < -40 || data.temperature > 125) {
                        Serial.println("[SDP810] ⚠ Temperature reading out of range!");
                    }
                }
            }
        }
        
        // Read other I2C sensors
        sensorManager->readSCD();
        sensorManager->readSFA1();
        sensorManager->readSFA2();
        sensorManager->readSGP1();
        sensorManager->readSGP2();
        sensorManager->readSHT1();
        sensorManager->readSHT2();
        
        // REMOVED: sensorManager->manageHeaters();
    }
    
    // Send data to ThingsBoard via MQTT Gateway
    if (thingsBoard && thingsBoard->isSendDue()) {
        if (thingsBoard->sendSensorData(*sensorManager)) {
            // Success logging is handled inside sendSensorData
        }
    }
    
    // Display data at intervals
    if (now - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = now;
        
        Serial.println("\n════════════════════════════════════════");
        Serial.printf("        Reading #%lu\n", lastDisplayTime / DISPLAY_INTERVAL);
        Serial.printf("  Time: %02d:%02d:%02d\n", 
                     (now / 3600000) % 24,
                     (now / 60000) % 60,
                     (now / 1000) % 60);
        Serial.printf("  Uptime: %.1f minutes\n", now / 60000.0);
        Serial.printf("  Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("  Compensation: %.1f°C, %.1f%% RH\n", 
                     sensorManager->getAverageTemperature(), 
                     sensorManager->getAverageHumidity());
        
        // SDP810 STATUS
        if (sdp810Verified) {
            Serial.println("  SDP810: ✓ Verified and active");
        } else if (sensorManager->isSDPActive()) {
            Serial.println("  SDP810: ⚠ Active but not verified");
        } else {
            Serial.println("  SDP810: ✗ Not detected");
        }
        
        // Show WiFi and MQTT status
        if (thingsBoard) {
            Serial.println("  WiFi: " + thingsBoard->getWiFiStatus());
            Serial.println("  Time Sync: " + String(thingsBoard->isTimeSync() ? "YES" : "NO"));
            
            // Check actual MQTT connection state
            if (thingsBoard->isMQTTConnected()) {  // CORRECTED: Use public method
                Serial.println("  MQTT: Connected (Persistent)");
                
                // Calculate next cloud upload time
                unsigned long timeSinceLastSend = now - thingsBoard->getLastSendTime();
                if (timeSinceLastSend < tbConfig.sendInterval) {
                    unsigned long timeToNextSend = tbConfig.sendInterval - timeSinceLastSend;
                    Serial.printf("  Next upload: %.1f seconds\n", timeToNextSend / 1000.0);
                } else {
                    Serial.println("  Next upload: Ready");
                }
            } else {
                Serial.println("  MQTT: Disconnected");
                Serial.println("  Next upload: Will reconnect");
            }
        } else {
            Serial.println("  System: Offline mode");
        }
        
        Serial.println("════════════════════════════════════════\n");
        
        // Display PMS5003 data
        if (sensorManager->isPMS1Active() && sensorManager->getPMS1Data().valid)
            printPMSData(sensorManager->getPMS1Data(), 1);
        if (sensorManager->isPMS2Active() && sensorManager->getPMS2Data().valid)
            printPMSData(sensorManager->getPMS2Data(), 2);
        
        Serial.println();
        
        // Display SDP810 data
        if (sdpOk) {
            printSDPData(sensorManager->getSDPData());
            sdpOk = false;
        } else if (sensorManager->isSDPActive() && sdp810Verified) {
            Serial.println("[SDP810] No new data");
        }
        
        Serial.println();
        
        // Display SCD40 data
        if (sensorManager->isSCDActive()) {
            printSCDData(sensorManager->getSCDData());
        }
        
        Serial.println();
        
        // Display SFA30 data
        if (sensorManager->isSFA1Active() && sensorManager->getSFA1Data().valid)
            printSFAData(sensorManager->getSFA1Data(), 1);
        if (sensorManager->isSFA2Active() && sensorManager->getSFA2Data().valid)
            printSFAData(sensorManager->getSFA2Data(), 2);
        
        Serial.println();
        
        // Display SGP41 data
        if (sensorManager->isSGP1Active())
            printSGPData(sensorManager->getSGP1Data(), 1);
        if (sensorManager->isSGP2Active())
            printSGPData(sensorManager->getSGP2Data(), 2);
        
        Serial.println();
        
        // Display SHT31 data
        if (sensorManager->isSHT1Active() && sensorManager->getSHT1Data().valid)
            printSHTData(sensorManager->getSHT1Data(), 1);
        if (sensorManager->isSHT2Active() && sensorManager->getSHT2Data().valid)
            printSHTData(sensorManager->getSHT2Data(), 2);
        
        Serial.println("\n──────────────────────────────────────");
        Serial.printf("Next local update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
        
        // Show sensor status every 5th reading
        static int readingCount = 0;
        readingCount++;
        if (readingCount % 5 == 0) {
            printSensorStatus();
        }
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}