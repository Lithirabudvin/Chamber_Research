#include <Arduino.h>
#include <Wire.h>
#include "PMS5003.h"
#include "SDP810.h"
#include <SensirionI2cScd4x.h>
#include <SensirionI2cSfa3x.h>

// PMS5003 Configuration
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// I2C Addresses
#define SDP810_I2C_ADDRESS 0x25
#define SCD40_I2C_ADDRESS 0x62
#define SFA30_I2C_ADDRESS 0x5D  // Use 0x5A if SEL pin is grounded

// Macro for error handling
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

// Create sensor objects
PMS5003 pmsSensor(Serial2, PMS_RX_PIN, PMS_TX_PIN);
SDP810 sdpSensor(Wire, SDP810_I2C_ADDRESS);
SensirionI2cScd4x scd40;
SensirionI2cSfa3x sfaSensor;

// Timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 5000; // Display every 5 seconds

// Sensor status
bool pmsActive = false;
bool sdpActive = false;
bool scdActive = false;
bool sfaActive = false;

// Error tracking
int scdErrorCount = 0;
int sfaErrorCount = 0;
const int MAX_ERRORS = 5;

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

SFA30Data sfaData = {0, 0, 0, false, 0};
SCD40Data scdData = {0, 0, 0, false, 0, false, 0};

// Function declarations
String getCO2Quality(uint16_t co2);
String getCO2Recommendation(uint16_t co2);
String getHCHOQuality(float hcho_ppb);
void printSensorStatus();
void printPMSData(const PMS5003::Data& data);
void printSDPData(const SDP810::Data& data);
void printSCDData(const SCD40Data& data);
void printSFAData(const SFA30Data& data);
void printOverallAirQuality(const PMS5003::Data& pmsData, const SCD40Data& scdData, const SFA30Data& sfaData);
bool initSCD40();
bool initSFA30();
void readSCD40();
void readSFA30();

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

void printSensorStatus() {
    Serial.println("\n════════════════════════════════════════");
    Serial.println("          SENSOR STATUS");
    Serial.println("════════════════════════════════════════");
    Serial.printf("  PMS5003:  %s\n", pmsActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SDP810:   %s\n", sdpActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SCD40:    %s (%d errors)\n", scdActive ? "✓ ACTIVE" : "✗ INACTIVE", scdData.errorCount);
    Serial.printf("  SFA30:    %s (%d errors)\n", sfaActive ? "✓ ACTIVE" : "✗ INACTIVE", sfaData.errorCount);
    Serial.println("════════════════════════════════════════\n");
}

bool initSCD40() {
    Serial.println("[SCD40] Initializing...");
    
    scd40.begin(Wire, SCD40_I2C_ADDRESS);
    delay(50);
    
    // Stop any ongoing measurement
    int16_t error = scd40.stopPeriodicMeasurement();
    if (error != NO_ERROR) {
        // This is OK if no measurement was running
    }
    delay(500);
    
    // Get serial number
    uint64_t serial = 0;
    error = scd40.getSerialNumber(serial);
    if (error == NO_ERROR) {
        Serial.print("  ✓ Detected! Serial: 0x");
        Serial.print((uint32_t)(serial >> 32), HEX);
        Serial.println((uint32_t)(serial & 0xFFFFFFFF), HEX);
        
        // Enable auto-calibration
        scd40.setAutomaticSelfCalibrationEnabled(true);
        
        // Start measurement
        error = scd40.startPeriodicMeasurement();
        if (error == NO_ERROR) {
            Serial.println("  ✓ Measurement started");
            return true;
        } else {
            Serial.println("  ✗ Failed to start measurement");
            return false;
        }
    } else {
        Serial.println("  ✗ Not detected!");
        return false;
    }
}

bool initSFA30() {
    Serial.println("[SFA30] Initializing...");
    
    sfaSensor.begin(Wire, SFA30_I2C_ADDRESS);
    delay(50);
    
    // Try to read device marking
    int8_t deviceMarking[32] = {0};
    int16_t error = sfaSensor.getDeviceMarking(deviceMarking, 32);
    
    if (error == NO_ERROR) {
        Serial.printf("  ✓ Detected! Device: %s\n", (const char*)deviceMarking);
        
        // Start continuous measurement
        error = sfaSensor.startContinuousMeasurement();
        if (error == NO_ERROR) {
            Serial.println("  ✓ Measurement started");
            return true;
        } else {
            Serial.println("  ✗ Failed to start measurement");
            return false;
        }
    } else {
        Serial.println("  ✗ Not detected!");
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  QUAD SENSOR AIR QUALITY MONITOR   ║");
    Serial.println("║ PMS5003 + SDP810 + SCD40 + SFA30   ║");
    Serial.println("╚══════════════════════════════════════╝\n");
    
    // ==================== I2C INITIALIZATION ====================
    Serial.println("[I2C] Initializing I2C bus...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);  // 100kHz standard speed
    delay(100);
    
    // ==================== PMS5003 INITIALIZATION ====================
    Serial.println("[PMS5003] Initializing...");
    if (pmsSensor.begin()) {
        pmsActive = true;
        Serial.println("  ✓ Sensor ready!");
    } else {
        Serial.println("  ✗ Initialization failed!");
    }
    
    // ==================== SDP810 INITIALIZATION ====================
    Serial.println("[SDP810] Initializing...");
    if (sdpSensor.begin()) {
        sdpActive = true;
        sdpSensor.startContinuousMeasurement(SDP810::CONTINUOUS_MEASUREMENT);
        Serial.println("  ✓ Continuous measurement started");
    } else {
        Serial.println("  ✗ Sensor not found!");
    }
    
    // ==================== SCD40 INITIALIZATION ====================
    scdActive = initSCD40();
    
    // ==================== SFA30 INITIALIZATION ====================
    sfaActive = initSFA30();
    
    // Display summary
    printSensorStatus();
    
    Serial.println("──────────────────────────────────────");
    Serial.println("IMPORTANT NOTES:");
    Serial.println("1. SCD40 needs ~30 seconds warm-up");
    Serial.println("2. SFA30 needs ~10 seconds warm-up");
    Serial.println("3. First readings may be inaccurate");
    Serial.println("──────────────────────────────────────");
    Serial.println("Starting measurements in 5 seconds...");
    delay(5000);
}

void readSCD40() {
    if (!scdActive) return;
    
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    
    // Check data ready status every 2 seconds
    if (now - lastCheck < 2000) return;
    lastCheck = now;
    
    // Check if data is ready
    bool dataReady = false;
    int16_t error = scd40.getDataReadyStatus(dataReady);
    
    if (error != NO_ERROR) {
        scdData.errorCount++;
        if (scdData.errorCount >= MAX_ERRORS) {
            scdActive = false;
            Serial.println("[SCD40] Too many errors, disabling sensor");
        }
        return;
    }
    
    scdData.dataReady = dataReady;
    
    if (dataReady) {
        // Data is ready, read it
        uint16_t co2 = 0;
        float temp = 0, hum = 0;
        
        error = scd40.readMeasurement(co2, temp, hum);
        
        if (error == NO_ERROR) {
            scdData.co2 = co2;
            scdData.temperature = temp;
            scdData.humidity = hum;
            scdData.valid = true;
            scdData.errorCount = 0; // Reset error count on success
            scdData.lastRead = now;
        } else {
            scdData.valid = false;
            scdData.errorCount++;
            
            char errorMessage[128];
            errorToString(error, errorMessage, sizeof(errorMessage));
            Serial.printf("[SCD40] Read error: %s (Count: %d)\n", errorMessage, scdData.errorCount);
            
            if (scdData.errorCount >= MAX_ERRORS) {
                scdActive = false;
                Serial.println("[SCD40] Too many errors, disabling sensor");
            }
        }
    }
}

void readSFA30() {
    if (!sfaActive) return;
    
    static unsigned long lastRead = 0;
    unsigned long now = millis();
    
    // Read SFA30 every 2 seconds
    if (now - lastRead < 2000) return;
    lastRead = now;
    
    float hcho = 0, humidity = 0, temperature = 0;
    int16_t error = sfaSensor.readMeasuredValues(hcho, humidity, temperature);
    
    if (error == NO_ERROR) {
        sfaData.formaldehyde = hcho;
        sfaData.humidity = humidity;
        sfaData.temperature = temperature;
        sfaData.valid = true;
        sfaData.errorCount = 0; // Reset error count on success
    } else {
        sfaData.valid = false;
        sfaData.errorCount++;
        
        char errorMessage[128];
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.printf("[SFA30] Read error: %s (Count: %d)\n", errorMessage, sfaData.errorCount);
        
        if (sfaData.errorCount >= MAX_ERRORS) {
            sfaActive = false;
            Serial.println("[SFA30] Too many errors, disabling sensor");
        }
    }
}

void printPMSData(const PMS5003::Data& data) {
    int aqi = pmsSensor.calculateAQI(data.pm25_standard);
    String category = pmsSensor.getAQICategory(aqi);
    
    Serial.println("┌────────────── PM SENSOR ──────────────┐");
    Serial.printf("│ PM1.0:  %3d μg/m³                     │\n", data.pm10_standard);
    Serial.printf("│ PM2.5:  %3d μg/m³                     │\n", data.pm25_standard);
    Serial.printf("│ PM10:   %3d μg/m³                     │\n", data.pm100_standard);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ AQI:    %3d (%s)          │\n", aqi, category.c_str());
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
        if (data.dataReady) {
            Serial.println("  (Data is ready but couldn't be read)");
        }
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

void printSFAData(const SFA30Data& data) {
    if (!data.valid) {
        Serial.println("[SFA30] No valid data yet");
        return;
    }
    
    String quality = getHCHOQuality(data.formaldehyde);
    float hcho_ppm = data.formaldehyde / 1000.0;
    
    Serial.println("┌──────────── FORMALDEHYDE ─────────────┐");
    Serial.printf("│ HCHO:    %6.1f ppb (%6.3f ppm)     │\n", data.formaldehyde, hcho_ppm);
    Serial.printf("│ RH:      %6.1f %%                   │\n", data.humidity);
    Serial.printf("│ Temp:    %6.1f °C                  │\n", data.temperature);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Quality: %-28s│\n", quality.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void loop() {
    static PMS5003::Data pmsData;
    static SDP810::Data sdpData;
    static bool pmsOk = false;
    static bool sdpOk = false;
    
    // ==================== READ SENSORS ====================
    
    // Read PMS5003
    if (pmsActive) {
        pmsOk = pmsSensor.readData(pmsData);
    }
    
    // Read SDP810
    if (sdpActive) {
        sdpOk = sdpSensor.readMeasurement(sdpData);
    }
    
    // Read I2C sensors (with error recovery)
    readSCD40();
    readSFA30();
    
    // ==================== DISPLAY DATA ====================
    if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        
        Serial.println("\n════════════════════════════════════════");
        Serial.printf("        Reading #%lu\n", lastDisplayTime / DISPLAY_INTERVAL);
        Serial.println("════════════════════════════════════════\n");
        
        // Display PMS5003 data
        if (pmsOk) {
            printPMSData(pmsData);
            pmsOk = false;
        } else if (pmsActive) {
            Serial.println("[PMS5003] No new data");
        }
        
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
        printSFAData(sfaData);
        
        Serial.println();
        
        // Display overall air quality if all sensors have data
        if (pmsActive && scdActive && sfaActive && pmsOk && scdData.valid && sfaData.valid) {
            printOverallAirQuality(pmsData, scdData, sfaData);
        }
        
        Serial.println("\n──────────────────────────────────────");
        Serial.printf("Next update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
        
        // Show sensor status every 5th reading
        static int readingCount = 0;
        readingCount++;
        if (readingCount % 5 == 0) {
            printSensorStatus();
        }
    }
    
    // Small delay to prevent CPU overload
    delay(100);
}

void printOverallAirQuality(const PMS5003::Data& pmsData, const SCD40Data& scdData, const SFA30Data& sfaData) {
    int pm25_aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
    String pm25_category = pmsSensor.getAQICategory(pm25_aqi);
    String co2_quality = getCO2Quality(scdData.co2);
    String hcho_quality = getHCHOQuality(sfaData.formaldehyde);
    
    // Determine overall rating
    String overall = "Moderate";
    
    // Simple logic - worst sensor determines overall
    if (pm25_aqi <= 50 && scdData.co2 < 800 && sfaData.formaldehyde < 80) {
        overall = "Good";
    } else if (pm25_aqi > 150 || scdData.co2 >= 1500 || sfaData.formaldehyde >= 200) {
        overall = "Poor";
    } else if (pm25_aqi > 100 || scdData.co2 >= 1000 || sfaData.formaldehyde >= 100) {
        overall = "Unhealthy for Sensitive";
    }
    
    Serial.println("┌───────── OVERALL AIR QUALITY ─────────┐");
    Serial.printf("│ PM2.5: %3d μg/m³ (%s)    │\n", pmsData.pm25_standard, pm25_category.c_str());
    Serial.printf("│ CO₂:   %5d ppm (%s)  │\n", scdData.co2, co2_quality.c_str());
    Serial.printf("│ HCHO:  %6.1f ppb (%s)  │\n", sfaData.formaldehyde, hcho_quality.c_str());
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Overall:   %-27s│\n", overall.c_str());
    Serial.println("└──────────────────────────────────────┘");
}