#include <Arduino.h>
#include "SensorManager.h"

// Configuration using constructor syntax
SensorManager::I2CConfig i2cConfig;
SensorManager::PMSConfig pmsConfig;

SensorManager sensorManager(i2cConfig, pmsConfig);

// Timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 5000;

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
    Serial.println("\n════════════════════════════════════════");
    Serial.println("          SENSOR STATUS");
    Serial.println("════════════════════════════════════════");
    Serial.printf("  PMS5003 #1: %s\n", sensorManager.isPMS1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  PMS5003 #2: %s\n", sensorManager.isPMS2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SDP810:     %s\n", sensorManager.isSDPActive() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SCD40:      %s\n", sensorManager.isSCDActive() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SFA30 #1:   %s\n", sensorManager.isSFA1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SFA30 #2:   %s\n", sensorManager.isSFA2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SGP41 #1:   %s", sensorManager.isSGP1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager.isSGP1Active() && sensorManager.getSGP1Data().conditioning) 
        Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.printf("  SGP41 #2:   %s", sensorManager.isSGP2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager.isSGP2Active() && sensorManager.getSGP2Data().conditioning) 
        Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.printf("  SHT31 #1:   %s", sensorManager.isSHT1Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager.isSHT1Active() && sensorManager.getSHT1Data().heaterEnabled) 
        Serial.print(" [HEATER ON]");
    Serial.println();
    Serial.printf("  SHT31 #2:   %s", sensorManager.isSHT2Active() ? "✓ ACTIVE" : "✗ INACTIVE");
    if (sensorManager.isSHT2Active() && sensorManager.getSHT2Data().heaterEnabled) 
        Serial.print(" [HEATER ON]");
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
    Serial.printf("│ Recommendation:%-22s│\n", recommendation.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void printSFAData(const SFA30::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[SFA30 #%d] No valid data\n", sensorNum);
        return;
    }
    
    String quality = getHCHOQuality(data.formaldehyde);
    float hcho_ppm = data.formaldehyde / 1000.0;
    
    Serial.printf("┌──────────── FORMALDEHYDE #%d ─────────────┐\n", sensorNum);
    Serial.printf("│ HCHO:    %6.1f ppb (%6.3f ppm)     │\n", data.formaldehyde, hcho_ppm);
    Serial.printf("│ RH:      %6.1f %%                   │\n", data.humidity);
    Serial.printf("│ Temp:    %6.1f °C                  │\n", data.temperature);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Quality: %-28s│\n", quality.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void printSGPData(const SGP41::Data& data, int sensorNum) {
    if (!data.valid) {
        if (data.conditioning) {
            Serial.printf("[SGP41 #%d] Conditioning\n", sensorNum);
        } else {
            Serial.printf("[SGP41 #%d] No valid data\n", sensorNum);
        }
        return;
    }
    
    String vocQuality = getVOCQuality(data.vocIndex);
    String noxQuality = getNOxQuality(data.noxIndex);
    
    Serial.printf("┌───────────── VOC/NOx SENSOR #%d ───────────┐\n", sensorNum);
    Serial.printf("│ VOC Raw:     %5d                   │\n", data.voc);
    Serial.printf("│ VOC Index:   %5.1f (%s)     │\n", data.vocIndex, vocQuality.c_str());
    Serial.printf("│ NOx Raw:     %5d                   │\n", data.nox);
    Serial.printf("│ NOx Index:   %5.1f (%s)     │\n", data.noxIndex, noxQuality.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void printSHTData(const SHT31::Data& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[SHT31 #%d] No valid data\n", sensorNum);
        return;
    }
    
    float dewPoint = calculateDewPoint(data.temperature, data.humidity);
    float heatIndex = calculateHeatIndex(data.temperature, data.humidity);
    float absHumidity = calculateAbsoluteHumidity(data.temperature, data.humidity);
    
    String tempQuality = getTemperatureQuality(data.temperature);
    String humQuality = getHumidityQuality(data.humidity);
    
    Serial.printf("┌─────────────── SHT31 SENSOR #%d ──────────────┐\n", sensorNum);
    Serial.printf("│ Temperature: %6.1f °C (%s)    │\n", data.temperature, tempQuality.c_str());
    Serial.printf("│ Humidity:    %6.1f %% (%s)      │\n", data.humidity, humQuality.c_str());
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Dew Point:   %6.1f °C                  │\n", dewPoint);
    Serial.printf("│ Heat Index:  %6.1f °C                  │\n", heatIndex);
    Serial.printf("│ Abs Humidity:%6.2f g/m³               │\n", absHumidity);
    Serial.printf("│ Heater:      %-28s│\n", data.heaterEnabled ? "ON" : "OFF");
    Serial.println("└──────────────────────────────────────┘");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  12-SENSOR AIR QUALITY MONITOR      ║");
    Serial.println("║  MODULAR VERSION                    ║");
    Serial.println("╚══════════════════════════════════════╝\n");
    
    // Initialize configurations
    i2cConfig.mainSDA = 21;
    i2cConfig.mainSCL = 22;
    i2cConfig.altSDA = 25;
    i2cConfig.altSCL = 26;
    
    pmsConfig.rxPin1 = 16;
    pmsConfig.txPin1 = 17;
    pmsConfig.rxPin2 = 18;
    pmsConfig.txPin2 = 19;
    
    Serial.println("════════════════════════════════════════");
    Serial.println("        SYSTEM CONFIGURATION");
    Serial.println("════════════════════════════════════════");
    Serial.println("Main I2C Bus (21,22):");
    Serial.println("  → SHT31 #1 (0x44), SHT31 #2 (0x45)");
    Serial.println("  → SCD40 (0x62), SFA30 #1 (0x5D)");
    Serial.println("  → SGP41 #1 (0x59), SGP41 #2 (0x59)");
    Serial.println("");
    Serial.println("Alternate Pins (25,26):");
    Serial.println("  → SFA30 #2 (0x5D) - Dynamic switching");
    Serial.println("");
    Serial.println("PMS5003 Sensors:");
    Serial.println("  → PMS5003 #1: RX=16, TX=17 (Serial2)");
    Serial.println("  → PMS5003 #2: RX=18, TX=19 (Serial1)");
    Serial.println("════════════════════════════════════════\n");
    
    // Initialize all sensors
    if (!sensorManager.begin()) {
        Serial.println("[ERROR] Failed to initialize sensors!");
        while (1) {
            Serial.println("System halted. Please check connections and restart.");
            delay(5000);
        }
    }
    
    // Print initial status
    printSensorStatus();
    
    Serial.println("Starting measurements in 3 seconds...");
    delay(3000);
}

void loop() {
    static bool sdpOk = false;
    static unsigned long lastReadTime = 0;
    
    unsigned long now = millis();
    
    // Read all sensors every 2 seconds
    if (now - lastReadTime >= 2000) {
        lastReadTime = now;
        
        // Read all sensors
        sensorManager.readPMS1();
        sensorManager.readPMS2();
        
        // Read SDP810
        if (sensorManager.isSDPActive()) {
            sdpOk = sensorManager.readSDP();
        }
        
        // Read I2C sensors
        sensorManager.readSCD();
        sensorManager.readSFA1();
        sensorManager.readSFA2();
        sensorManager.readSGP1();
        sensorManager.readSGP2();
        sensorManager.readSHT1();
        sensorManager.readSHT2();
        
        // Manage SHT31 heaters
        sensorManager.manageHeaters();
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
        Serial.printf("  Compensation: %.1f°C, %.1f%% RH\n", 
                     sensorManager.getAverageTemperature(), 
                     sensorManager.getAverageHumidity());
        Serial.println("════════════════════════════════════════\n");
        
        // Display PMS5003 data
        if (sensorManager.isPMS1Active() && sensorManager.getPMS1Data().valid)
            printPMSData(sensorManager.getPMS1Data(), 1);
        if (sensorManager.isPMS2Active() && sensorManager.getPMS2Data().valid)
            printPMSData(sensorManager.getPMS2Data(), 2);
        
        Serial.println();
        
        // Display SDP810 data
        if (sdpOk) {
            printSDPData(sensorManager.getSDPData());
            sdpOk = false;
        } else if (sensorManager.isSDPActive()) {
            Serial.println("[SDP810] No new data");
        }
        
        Serial.println();
        
        // Display SCD40 data
        if (sensorManager.isSCDActive()) {
            printSCDData(sensorManager.getSCDData());
        }
        
        Serial.println();
        
        // Display SFA30 data
        if (sensorManager.isSFA1Active() && sensorManager.getSFA1Data().valid)
            printSFAData(sensorManager.getSFA1Data(), 1);
        if (sensorManager.isSFA2Active() && sensorManager.getSFA2Data().valid)
            printSFAData(sensorManager.getSFA2Data(), 2);
        
        Serial.println();
        
        // Display SGP41 data
        if (sensorManager.isSGP1Active())
            printSGPData(sensorManager.getSGP1Data(), 1);
        if (sensorManager.isSGP2Active())
            printSGPData(sensorManager.getSGP2Data(), 2);
        
        Serial.println();
        
        // Display SHT31 data
        if (sensorManager.isSHT1Active() && sensorManager.getSHT1Data().valid)
            printSHTData(sensorManager.getSHT1Data(), 1);
        if (sensorManager.isSHT2Active() && sensorManager.getSHT2Data().valid)
            printSHTData(sensorManager.getSHT2Data(), 2);
        
        Serial.println("\n──────────────────────────────────────");
        Serial.printf("Next update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
        
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