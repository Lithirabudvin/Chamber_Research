#include <Arduino.h>
#include <Wire.h>
#include "PMS5003.h"
#include "SDP810.h"
#include <SensirionI2cScd4x.h>
#include <SensirionI2cSfa3x.h>
#include <SensirionI2CSgp41.h>
#include <Adafruit_SHT31.h>

// PMS5003 Configuration (2 sensors)
#define PMS_RX_PIN_1 16
#define PMS_TX_PIN_1 17
#define PMS_RX_PIN_2 18
#define PMS_TX_PIN_2 19

// I2C Pins - Dynamic switching
#define MAIN_SDA 21
#define MAIN_SCL 22
#define ALT_SDA 25    // For SFA30 #2
#define ALT_SCL 26    // For SFA30 #2
// I2C Addresses
#define SDP810_I2C_ADDRESS 0x25
#define SCD40_I2C_ADDRESS 0x62
#define SFA30_I2C_ADDRESS 0x5D    // Fixed address for ALL SFA30 sensors
#define SGP41_I2C_ADDRESS 0x59    // Fixed address for ALL SGP41 sensors
#define SHT31_I2C_ADDRESS_1 0x44  // First SHT31 (ADDR to GND)
#define SHT31_I2C_ADDRESS_2 0x45  // Second SHT31 (ADDR to 3.3V)

// Macro for error handling
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

// Create sensor objects
PMS5003 pmsSensor1(Serial2, PMS_RX_PIN_1, PMS_TX_PIN_1);
PMS5003 pmsSensor2(Serial1, PMS_RX_PIN_2, PMS_TX_PIN_2);
SDP810 sdpSensor(Wire, SDP810_I2C_ADDRESS);
SensirionI2cScd4x scd40;
SensirionI2cSfa3x sfaSensor1;
SensirionI2cSfa3x sfaSensor2;
SensirionI2CSgp41 sgp41_1;
SensirionI2CSgp41 sgp41_2;
Adafruit_SHT31 sht31_1 = Adafruit_SHT31();
Adafruit_SHT31 sht31_2 = Adafruit_SHT31();

// Timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 5000;

// Sensor status
bool pmsActive1 = false;
bool pmsActive2 = false;
bool sdpActive = false;
bool scdActive = false;
bool sfaActive1 = false;
bool sfaActive2 = false;
bool sgpActive1 = false;
bool sgpActive2 = false;
bool shtActive1 = false;
bool shtActive2 = false;

// Error tracking
const int MAX_ERRORS = 10;  // Increased for PMS sensors

// Data structures
struct SFA30Data {
    float formaldehyde;  // ppb
    float humidity;      // %RH
    float temperature;   // °C
    bool valid;
    int errorCount;
};

struct SCD40Data {
    uint16_t co2;        // ppm
    float temperature;   // °C
    float humidity;      // %RH
    bool valid;
    int errorCount;
    bool dataReady;
    unsigned long lastRead;
};

struct SGP41Data {
    uint16_t voc;        // VOC signal (raw)
    uint16_t nox;        // NOx signal (raw)
    float vocIndex;      // Calculated VOC index (0-500)
    float noxIndex;      // Calculated NOx index (1-5)
    bool valid;
    int errorCount;
    bool conditioning;   // True during first 10 seconds
};

struct SHT31Data {
    float temperature;   // °C
    float humidity;      // %RH
    bool heaterEnabled;  // Heater status
    bool valid;
    int errorCount;
};

struct PMSData {
    uint16_t pm10_standard;
    uint16_t pm25_standard;
    uint16_t pm100_standard;
    bool valid;
    int errorCount;
};

// Sensor data instances
PMSData pmsData1 = {0, 0, 0, false, 0};
PMSData pmsData2 = {0, 0, 0, false, 0};
SFA30Data sfaData1 = {0, 0, 0, false, 0};
SFA30Data sfaData2 = {0, 0, 0, false, 0};
SCD40Data scdData = {0, 0, 0, false, 0, false, 0};
SGP41Data sgpData1 = {0, 0, 0.0, 0.0, false, 0, true};
SGP41Data sgpData2 = {0, 0, 0.0, 0.0, false, 0, true};
SHT31Data shtData1 = {0, 0, false, false, 0};
SHT31Data shtData2 = {0, 0, false, false, 0};

// SGP41 conditioning times
uint16_t sgpConditioningTime1 = 10;
uint16_t sgpConditioningTime2 = 10;

// SHT31 heater control
unsigned long lastHeaterToggle1 = 0;
unsigned long lastHeaterToggle2 = 0;
bool shtHeaterEnabled1 = false;
bool shtHeaterEnabled2 = false;
const unsigned long HEATER_INTERVAL = 30000;

// Function declarations
String getCO2Quality(uint16_t co2);
String getCO2Recommendation(uint16_t co2);
String getHCHOQuality(float hcho_ppb);
String getVOCQuality(float vocIndex);
String getNOxQuality(float noxIndex);
String getTemperatureQuality(float temp);
String getHumidityQuality(float humidity);
void printSensorStatus();
void printPMSData(const PMSData& data, int sensorNum);
void printSDPData(const SDP810::Data& data);
void printSCDData(const SCD40Data& data);
void printSFAData(const SFA30Data& data, int sensorNum);
void printSGPData(const SGP41Data& data, int sensorNum);
void printSHTData(const SHT31Data& data, int sensorNum);
bool initSFA30_AltPins(SensirionI2cSfa3x& sensor, int sensorNum);
void readSFA30_AltPins(SensirionI2cSfa3x& sensor, SFA30Data& data, bool& active, int sensorNum);
float calculateVOCIndex(uint16_t rawVoc, float vocBaseline = 100.0);
float calculateNOxIndex(uint16_t rawNox, float noxBaseline = 100.0);
float calculateDewPoint(float temp, float humidity);
float calculateHeatIndex(float temp, float humidity);
float calculateAbsoluteHumidity(float temp, float humidity);
void readPMS5003(PMS5003& sensor, PMSData& data, bool& active, int sensorNum);
void readSCD40();
void readSGP41(SensirionI2CSgp41& sensor, SGP41Data& data, bool& active, int sensorNum, uint16_t& conditioningTime, float temp, float humidity);
void readSHT31(Adafruit_SHT31& sensor, SHT31Data& data, bool& active, int sensorNum);
void manageSHT31Heater(Adafruit_SHT31& sensor, SHT31Data& data, unsigned long& lastHeaterToggle, bool& heaterEnabled);
uint16_t convertTemperature(float temperature);
uint16_t convertHumidity(float humidity);

// Convert temperature to ticks (SGP41 format)
uint16_t convertTemperature(float temperature) {
    int32_t ticks = (int32_t)((temperature + 45) * 65535.0 / 175.0);
    if (ticks > 65535) ticks = 65535;
    if (ticks < 0) ticks = 0;
    return (uint16_t)ticks;
}

// Convert humidity to ticks (SGP41 format)
uint16_t convertHumidity(float humidity) {
    int32_t ticks = (int32_t)(humidity * 65535.0 / 100.0);
    if (ticks > 65535) ticks = 65535;
    if (ticks < 0) ticks = 0;
    return (uint16_t)ticks;
}

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

// FIXED VOC Index calculation
float calculateVOCIndex(uint16_t rawVoc, float vocBaseline) {
    // Correct calculation: Remove *100.0
    if (rawVoc > 65000) return 500.0;
    return (rawVoc / 650.0);  // Should give ~45 for 30000
}

// FIXED NOx Index calculation
float calculateNOxIndex(uint16_t rawNox, float noxBaseline) {
    // Correct calculation: Remove *4.0
    if (rawNox > 65000) return 5.0;
    return 1.0 + (rawNox / 13000.0);  // Should give ~2-3 for 18000
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

void printSensorStatus() {
    Serial.println("\n════════════════════════════════════════");
    Serial.println("          SENSOR STATUS");
    Serial.println("════════════════════════════════════════");
    Serial.printf("  PMS5003 #1: %s\n", pmsActive1 ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  PMS5003 #2: %s\n", pmsActive2 ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SDP810:     %s\n", sdpActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SCD40:      %s (%d errors)\n", scdActive ? "✓ ACTIVE" : "✗ INACTIVE", scdData.errorCount);
    Serial.printf("  SFA30 #1:   %s (%d errors)\n", sfaActive1 ? "✓ ACTIVE" : "✗ INACTIVE", sfaData1.errorCount);
    Serial.printf("  SFA30 #2:   %s (%d errors)\n", sfaActive2 ? "✓ ACTIVE" : "✗ INACTIVE", sfaData2.errorCount);
    Serial.printf("  SGP41 #1:   %s (%d errors)", sgpActive1 ? "✓ ACTIVE" : "✗ INACTIVE", sgpData1.errorCount);
    if (sgpData1.conditioning) Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.printf("  SGP41 #2:   %s (%d errors)", sgpActive2 ? "✓ ACTIVE" : "✗ INACTIVE", sgpData2.errorCount);
    if (sgpData2.conditioning) Serial.print(" [CONDITIONING]");
    Serial.println();
    Serial.printf("  SHT31 #1:   %s (%d errors)", shtActive1 ? "✓ ACTIVE" : "✗ INACTIVE", shtData1.errorCount);
    if (shtData1.heaterEnabled) Serial.print(" [HEATER ON]");
    Serial.println();
    Serial.printf("  SHT31 #2:   %s (%d errors)", shtActive2 ? "✓ ACTIVE" : "✗ INACTIVE", shtData2.errorCount);
    if (shtData2.heaterEnabled) Serial.print(" [HEATER ON]");
    Serial.println();
    Serial.println("════════════════════════════════════════\n");
}

void printPMSData(const PMSData& data, int sensorNum) {
    if (!data.valid) {
        Serial.printf("[PMS5003 #%d] No valid data\n", sensorNum);
        return;
    }
    
    Serial.printf("┌────────────── PM SENSOR #%d ──────────────┐\n", sensorNum);
    Serial.printf("│ PM1.0:  %3d μg/m³                     │\n", data.pm10_standard);
    Serial.printf("│ PM2.5:  %3d μg/m³                     │\n", data.pm25_standard);
    Serial.printf("│ PM10:   %3d μg/m³                     │\n", data.pm100_standard);
    Serial.println("└──────────────────────────────────────┘");
}

void printSDPData(const SDP810::Data& data) {
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

void printSCDData(const SCD40Data& data) {
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

void printSFAData(const SFA30Data& data, int sensorNum) {
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

void printSGPData(const SGP41Data& data, int sensorNum) {
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

void printSHTData(const SHT31Data& data, int sensorNum) {
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

// SFA30 #2 initialization with alternate pins
bool initSFA30_AltPins(SensirionI2cSfa3x& sensor, int sensorNum) {
    Serial.printf("[SFA30 #%d] Initializing on alternate pins (25,26)...\n", sensorNum);
    
    // Temporarily switch to alternate pins
    Wire.end();
    delay(10);
    Wire.begin(ALT_SDA, ALT_SCL);
    delay(50);
    
    sensor.begin(Wire, SFA30_I2C_ADDRESS);
    delay(50);
    
    int8_t deviceMarking[32] = {0};
    int16_t error = sensor.getDeviceMarking(deviceMarking, 32);
    
    if (error == NO_ERROR) {
        Serial.printf("  ✓ Detected! Device: %s\n", (const char*)deviceMarking);
        
        error = sensor.startContinuousMeasurement();
        if (error == NO_ERROR) {
            Serial.println("  ✓ Measurement started");
            
            // Switch back to main pins
            Wire.end();
            delay(10);
            Wire.begin(MAIN_SDA, MAIN_SCL);
            return true;
        }
    }
    Serial.printf("  ✗ Not detected for sensor #%d\n", sensorNum);
    
    // Switch back to main pins
    Wire.end();
    delay(10);
    Wire.begin(MAIN_SDA, MAIN_SCL);
    return false;
}

// Read SFA30 #2 with alternate pins
void readSFA30_AltPins(SensirionI2cSfa3x& sensor, SFA30Data& data, bool& active, int sensorNum) {
    if (!active) return;
    
    static unsigned long lastRead = 0;
    unsigned long now = millis();
    
    if (now - lastRead < 2000) return;
    lastRead = now;
    
    // Switch to alternate pins
    Wire.end();
    delay(5);
    Wire.begin(ALT_SDA, ALT_SCL);
    delay(5);
    
    // Reinitialize sensor with new bus
    sensor.begin(Wire, SFA30_I2C_ADDRESS);
    delay(5);
    
    float hcho = 0, humidity = 0, temperature = 0;
    int16_t error = sensor.readMeasuredValues(hcho, humidity, temperature);
    
    // Switch back to main pins immediately
    Wire.end();
    delay(5);
    Wire.begin(MAIN_SDA, MAIN_SCL);
    
    if (error == NO_ERROR) {
        data.formaldehyde = hcho;
        data.humidity = humidity;
        data.temperature = temperature;
        data.valid = true;
        data.errorCount = 0;
    } else {
        data.valid = false;
        data.errorCount++;
        if (data.errorCount >= MAX_ERRORS) {
            active = false;
            Serial.printf("[SFA30 #%d] Too many errors, disabling\n", sensorNum);
        }
    }
}

// PMS5003 reading with improved error handling
void readPMS5003(PMS5003& sensor, PMSData& data, bool& active, int sensorNum) {
    if (!active) return;
    
    static unsigned long lastRead[2] = {0, 0};
    static unsigned long lastSuccess[2] = {0, 0};
    unsigned long now = millis();
    
    // Read every 2 seconds (PMS sends data every 1 second)
    if (now - lastRead[sensorNum-1] < 2000) return;
    lastRead[sensorNum-1] = now;
    
    PMS5003::Data rawData;
    
    // METHOD 1: Try non-blocking first (recommended)
    if (sensor.readDataNonBlocking(rawData)) {
        data.pm10_standard = rawData.pm10_standard;
        data.pm25_standard = rawData.pm25_standard;
        data.pm100_standard = rawData.pm100_standard;
        data.valid = true;
        data.errorCount = 0;
        lastSuccess[sensorNum-1] = now;
        
        Serial.printf("[PMS%d] ✓ PM2.5=%d, PM10=%d\n", 
                     sensorNum, rawData.pm25_standard, rawData.pm100_standard);
    }
    // METHOD 2: Try with timeout (100ms max)
    else if (sensor.readData(rawData, 100)) {
        data.pm10_standard = rawData.pm10_standard;
        data.pm25_standard = rawData.pm25_standard;
        data.pm100_standard = rawData.pm100_standard;
        data.valid = true;
        data.errorCount = 0;
        lastSuccess[sensorNum-1] = now;
        
        Serial.printf("[PMS%d] ⏱ PM2.5=%d (timeout used)\n", 
                     sensorNum, rawData.pm25_standard);
    }
    else {
        data.valid = false;
        data.errorCount++;
        
        // If no success for 10 seconds, wake sensor
        if (now - lastSuccess[sensorNum-1] > 10000) {
            Serial.printf("[PMS%d] No data for 10s - waking...\n", sensorNum);
            sensor.wakeUp();
            delay(50);
            lastSuccess[sensorNum-1] = now;
            data.errorCount = 0;
        }
        
        Serial.printf("[PMS%d] ✗ No data (errors: %d)\n", 
                     sensorNum, data.errorCount);
        
        if (data.errorCount >= 10) {
            active = false;
            Serial.printf("[PMS%d] DISABLED\n", sensorNum);
        }
    }
}

void readSCD40() {
    if (!scdActive) return;
    
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    
    if (now - lastCheck < 2000) return;
    lastCheck = now;
    
    bool dataReady = false;
    int16_t error = scd40.getDataReadyStatus(dataReady);
    
    if (error != NO_ERROR) {
        scdData.errorCount++;
        if (scdData.errorCount >= MAX_ERRORS) scdActive = false;
        return;
    }
    
    scdData.dataReady = dataReady;
    
    if (dataReady) {
        uint16_t co2 = 0;
        float temp = 0, hum = 0;
        
        error = scd40.readMeasurement(co2, temp, hum);
        
        if (error == NO_ERROR) {
            scdData.co2 = co2;
            scdData.temperature = temp;
            scdData.humidity = hum;
            scdData.valid = true;
            scdData.errorCount = 0;
            scdData.lastRead = now;
        } else {
            scdData.valid = false;
            scdData.errorCount++;
            if (scdData.errorCount >= MAX_ERRORS) scdActive = false;
        }
    }
}

void readSGP41(SensirionI2CSgp41& sensor, SGP41Data& data, bool& active, int sensorNum, uint16_t& conditioningTime, float temp, float humidity) {
    if (!active) return;
    
    static unsigned long lastRead1 = 0;
    static unsigned long lastRead2 = 0;
    unsigned long* lastRead = (sensorNum == 1) ? &lastRead1 : &lastRead2;
    unsigned long now = millis();
    
    if (now - *lastRead < 2000) return;
    *lastRead = now;
    
    uint16_t rh_ticks = convertHumidity(humidity);
    uint16_t t_ticks = convertTemperature(temp);
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    
    int16_t error;
    
    if (data.conditioning && conditioningTime > 0) {
        error = sensor.executeConditioning(rh_ticks, t_ticks, srawVoc);
        conditioningTime--;
        
        if (conditioningTime == 0) {
            data.conditioning = false;
            Serial.printf("[SGP41 #%d] NOx conditioning complete!\n", sensorNum);
        }
    } else {
        error = sensor.measureRawSignals(rh_ticks, t_ticks, srawVoc, srawNox);
    }
    
    if (error == NO_ERROR) {
        data.voc = srawVoc;
        data.nox = srawNox;
        data.vocIndex = calculateVOCIndex(srawVoc);   // Fixed calculation
        data.noxIndex = calculateNOxIndex(srawNox);   // Fixed calculation
        data.valid = true;
        data.errorCount = 0;
    } else {
        data.valid = false;
        data.errorCount++;
        if (data.errorCount >= MAX_ERRORS) {
            active = false;
            Serial.printf("[SGP41 #%d] Too many errors, disabling\n", sensorNum);
        }
    }
}

void readSHT31(Adafruit_SHT31& sensor, SHT31Data& data, bool& active, int sensorNum) {
    if (!active) return;
    
    static unsigned long lastRead1 = 0;
    static unsigned long lastRead2 = 0;
    unsigned long* lastRead = (sensorNum == 1) ? &lastRead1 : &lastRead2;
    unsigned long now = millis();
    
    if (now - *lastRead < 2000) return;
    *lastRead = now;
    
    float temperature = sensor.readTemperature();
    float humidity = sensor.readHumidity();
    
    if (!isnan(temperature) && !isnan(humidity)) {
        data.temperature = temperature;
        data.humidity = humidity;
        data.heaterEnabled = sensor.isHeaterEnabled();
        data.valid = true;
        data.errorCount = 0;
    } else {
        data.valid = false;
        data.errorCount++;
        
        if (data.errorCount >= MAX_ERRORS) {
            active = false;
            Serial.printf("[SHT31 #%d] Too many errors, disabling\n", sensorNum);
        }
    }
}

void manageSHT31Heater(Adafruit_SHT31& sensor, SHT31Data& data, unsigned long& lastHeaterToggle, bool& heaterEnabled) {
    unsigned long now = millis();
    
    if (now - lastHeaterToggle >= HEATER_INTERVAL) {
        lastHeaterToggle = now;
        heaterEnabled = !heaterEnabled;
        sensor.heater(heaterEnabled);
        data.heaterEnabled = heaterEnabled;
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  12-SENSOR AIR QUALITY MONITOR      ║");
    Serial.println("║  FINAL WORKING VERSION              ║");
    Serial.println("╚══════════════════════════════════════╝\n");
    
    // Initialize main I2C bus
    Serial.println("[I2C] Initializing main I2C bus on pins 21,22...");
    Wire.begin(MAIN_SDA, MAIN_SCL);
    Wire.setClock(100000);
    delay(100);
    
    // ==================== PMS5003 INITIALIZATION ====================
    Serial.println("[PMS5003 #1] Initializing...");
    Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN_1, PMS_TX_PIN_1);
    delay(100);
    if (pmsSensor1.begin()) {
        pmsActive1 = true;
        Serial.println("  ✓ Sensor ready!");
    } else {
        Serial.println("  ✗ Initialization failed!");
    }
    
    Serial.println("[PMS5003 #2] Initializing...");
    Serial1.begin(9600, SERIAL_8N1, PMS_RX_PIN_2, PMS_TX_PIN_2);
    delay(100);
    // REMOVED TIMEOUT PARAMETER
    if (pmsSensor2.begin()) {
        pmsActive2 = true;
        Serial.println("  ✓ Sensor ready!");
    } else {
        Serial.println("  ✗ Initialization failed!");
    }
    // ==================== SDP810 INITIALIZATION ====================
    Serial.println("\n[SDP810] Scanning for address...");
    sdpActive = false;
    uint8_t sdpAddresses[] = {0x25, 0x26, 0x27, 0x28};
    for (int i = 0; i < 4; i++) {
        Serial.printf("  Trying 0x%02X... ", sdpAddresses[i]);
        Wire.beginTransmission(sdpAddresses[i]);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.println("Found!");
            // Initialize SDP810 without address parameter
            if (sdpSensor.begin()) {
                sdpActive = true;
                sdpSensor.startContinuousMeasurement(SDP810::CONTINUOUS_MEASUREMENT);
                Serial.println("  ✓ Measurement started");
                break;
            }
        } else {
            Serial.println("Not found");
        }
    }
    
    if (!sdpActive) {
        Serial.println("  ✗ SDP810 not detected!");
    }
    
    // ==================== SCD40 INITIALIZATION ====================
    Serial.println("[SCD40] Initializing...");
    scd40.begin(Wire, SCD40_I2C_ADDRESS);
    delay(50);
    
    int16_t error = scd40.stopPeriodicMeasurement();
    delay(500);
    
    uint64_t serial = 0;
    error = scd40.getSerialNumber(serial);
    if (error == NO_ERROR) {
        scdActive = true;
        Serial.print("  ✓ Detected! Serial: 0x");
        Serial.print((uint32_t)(serial >> 32), HEX);
        Serial.println((uint32_t)(serial & 0xFFFFFFFF), HEX);
        
        scd40.setAutomaticSelfCalibrationEnabled(true);
        error = scd40.startPeriodicMeasurement();
        if (error == NO_ERROR) {
            Serial.println("  ✓ Measurement started");
        }
    } else {
        Serial.println("  ✗ Not detected!");
    }
    
    // ==================== SHT31 INITIALIZATION ====================
    Serial.println("[SHT31] Initializing...");
    shtActive1 = sht31_1.begin(SHT31_I2C_ADDRESS_1);
    if (shtActive1) {
        Serial.println("  ✓ SHT31 #1 detected");
        sht31_1.heater(false);
    } else {
        Serial.println("  ✗ SHT31 #1 not found");
    }
    
    shtActive2 = sht31_2.begin(SHT31_I2C_ADDRESS_2);
    if (shtActive2) {
        Serial.println("  ✓ SHT31 #2 detected");
        sht31_2.heater(false);
    } else {
        Serial.println("  ✗ SHT31 #2 not found");
    }
    
    // ==================== SFA30 INITIALIZATION ====================
    Serial.println("[SFA30 #1] Initializing on main bus...");
    sfaSensor1.begin(Wire, SFA30_I2C_ADDRESS);
    delay(50);
    
    int8_t deviceMarking1[32] = {0};
    error = sfaSensor1.getDeviceMarking(deviceMarking1, 32);
    if (error == NO_ERROR) {
        sfaActive1 = true;
        Serial.printf("  ✓ Detected! Device: %s\n", (const char*)deviceMarking1);
        sfaSensor1.startContinuousMeasurement();
        Serial.println("  ✓ Measurement started");
    } else {
        Serial.println("  ✗ Not detected!");
    }
    
    // SFA30 #2 on alternate pins
    sfaActive2 = initSFA30_AltPins(sfaSensor2, 2);
    
    // ==================== SGP41 INITIALIZATION ====================
    Serial.println("[SGP41 #1] Initializing...");
    sgp41_1.begin(Wire);
    delay(50);
    
    uint8_t serialNumberSize = 3;
    uint16_t serialNumber1[serialNumberSize];
    error = sgp41_1.getSerialNumber(serialNumber1);
    if (error == NO_ERROR) {
        sgpActive1 = true;
        Serial.print("  ✓ Detected! Serial: 0x");
        for (size_t i = 0; i < serialNumberSize; i++) {
            Serial.print(serialNumber1[i], HEX);
        }
        Serial.println();
    } else {
        Serial.println("  ✗ Not detected!");
    }
    
    Serial.println("[SGP41 #2] Initializing...");
    sgp41_2.begin(Wire);
    delay(50);
    
    uint16_t serialNumber2[serialNumberSize];
    error = sgp41_2.getSerialNumber(serialNumber2);
    if (error == NO_ERROR) {
        sgpActive2 = true;
        Serial.print("  ✓ Detected! Serial: 0x");
        for (size_t i = 0; i < serialNumberSize; i++) {
            Serial.print(serialNumber2[i], HEX);
        }
        Serial.println();
    } else {
        Serial.println("  ✗ Not detected!");
    }
    
    // ==================== FINAL STATUS ====================
    printSensorStatus();
    
    Serial.println("════════════════════════════════════════");
    Serial.println("      SYSTEM CONFIGURATION");
    Serial.println("════════════════════════════════════════");
    Serial.println("Main I2C Bus (21,22):");
    Serial.println("  → SHT31 #1 (0x44), SHT31 #2 (0x45)");
    Serial.println("  → SCD40 (0x62), SFA30 #1 (0x5D)");
    Serial.println("  → SGP41 #1 (0x59), SGP41 #2 (0x59)");
    Serial.println("");
    Serial.println("Alternate Pins (25,26):");
    Serial.println("  → SFA30 #2 (0x5D) - Dynamic switching");
    Serial.println("════════════════════════════════════════\n");
    
    Serial.println("Starting measurements in 5 seconds...");
    delay(5000);
}

void loop() {
    static SDP810::Data sdpData;
    static bool sdpOk = false;
    
    // Get temperature and humidity from SHT31 sensors for SGP41 compensation
    float avgTemp = (shtData1.temperature + shtData2.temperature) / 2.0;
    float avgHumidity = (shtData1.humidity + shtData2.humidity) / 2.0;
    
    // Default values if SHT31 not ready
    if (isnan(avgTemp) || avgTemp == 0) avgTemp = 25.0;
    if (isnan(avgHumidity) || avgHumidity == 0) avgHumidity = 50.0;
    
    // ==================== READ SENSORS ====================
    
    // Read PMS5003 sensors
    readPMS5003(pmsSensor1, pmsData1, pmsActive1, 1);
    readPMS5003(pmsSensor2, pmsData2, pmsActive2, 2);
    
    // Read SDP810
    if (sdpActive) {
        sdpOk = sdpSensor.readMeasurement(sdpData);
    }
    
    // Read I2C sensors
    readSCD40();
    
    // Read SFA30 #1 (main bus)
    float hcho1 = 0, humidity1 = 0, temperature1 = 0;
    if (sfaActive1) {
        int16_t error = sfaSensor1.readMeasuredValues(hcho1, humidity1, temperature1);
        if (error == NO_ERROR) {
            sfaData1.formaldehyde = hcho1;
            sfaData1.humidity = humidity1;
            sfaData1.temperature = temperature1;
            sfaData1.valid = true;
        }
    }
    
    // Read SFA30 #2 (alternate pins)
    readSFA30_AltPins(sfaSensor2, sfaData2, sfaActive2, 2);
    
    // Read SGP41 sensors
    readSGP41(sgp41_1, sgpData1, sgpActive1, 1, sgpConditioningTime1, avgTemp, avgHumidity);
    readSGP41(sgp41_2, sgpData2, sgpActive2, 2, sgpConditioningTime2, avgTemp, avgHumidity);
    
    // Read SHT31 sensors
    readSHT31(sht31_1, shtData1, shtActive1, 1);
    readSHT31(sht31_2, shtData2, shtActive2, 2);
    
    // Manage SHT31 heaters
    manageSHT31Heater(sht31_1, shtData1, lastHeaterToggle1, shtHeaterEnabled1);
    manageSHT31Heater(sht31_2, shtData2, lastHeaterToggle2, shtHeaterEnabled2);
    
    // ==================== DISPLAY DATA ====================
    if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        
        Serial.println("\n════════════════════════════════════════");
        Serial.printf("        Reading #%lu\n", lastDisplayTime / DISPLAY_INTERVAL);
        Serial.printf("  Compensation: %.1f°C, %.1f%% RH\n", avgTemp, avgHumidity);
        Serial.println("════════════════════════════════════════\n");
        
        // Display PMS5003 data
        if (pmsActive1 && pmsData1.valid) printPMSData(pmsData1, 1);
        if (pmsActive2 && pmsData2.valid) printPMSData(pmsData2, 2);
        
        Serial.println();
        
        // Display SDP810 data
        if (sdpOk) {
            printSDPData(sdpData);
            sdpOk = false;
        } else if (sdpActive) {
            Serial.println("[SDP810] No new data");
        }
        
        Serial.println();
        
        // Display SCD40 data
        printSCDData(scdData);
        
        Serial.println();
        
        // Display SFA30 data
        if (sfaActive1 && sfaData1.valid) printSFAData(sfaData1, 1);
        if (sfaActive2 && sfaData2.valid) printSFAData(sfaData2, 2);
        
        Serial.println();
        
        // Display SGP41 data
        if (sgpActive1) printSGPData(sgpData1, 1);
        if (sgpActive2) printSGPData(sgpData2, 2);
        
        Serial.println();
        
        // Display SHT31 data
        if (shtActive1 && shtData1.valid) printSHTData(shtData1, 1);
        if (shtActive2 && shtData2.valid) printSHTData(shtData2, 2);
        
        Serial.println("\n──────────────────────────────────────");
        Serial.printf("Next update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
        
        // Show sensor status every 5th reading
        static int readingCount = 0;
        readingCount++;
        if (readingCount % 5 == 0) {
            printSensorStatus();
        }
    }
    
    delay(100);
}