#include <Arduino.h>
#include <Wire.h>
#include "PMS5003.h"
#include "SDP810.h"
#include <SensirionI2cSfa3x.h>

// Sensor Pin Definitions
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

#define I2C_SDA 21
#define I2C_SCL 22
#define SDP810_I2C_ADDRESS 0x25
#define SFA30_I2C_ADDRESS SFA3X_I2C_ADDR_5D  // 0x5D

// Sensor Objects
PMS5003 pmsSensor(Serial2, PMS_RX_PIN, PMS_TX_PIN);
SDP810 sdpSensor(Wire, SDP810_I2C_ADDRESS);
SensirionI2cSfa3x sfaSensor;

// Error handling
static char errorMessage[64];
static int16_t error;
#define NO_ERROR 0

// Display timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 5000; // Display every 5 seconds

// Sensor status flags
bool pmsActive = false;
bool sdpActive = false;
bool sfaActive = false;

// Formaldehyde quality helper
String getHCHOQualityLabel(float hcho_ppb) {
    float hcho_ppm = hcho_ppb / 1000.0; // Convert ppb to ppm
    
    // WHO guidelines for formaldehyde:
    if (hcho_ppm < 0.08f) return "Good";
    else if (hcho_ppm < 0.1f) return "Moderate";
    else if (hcho_ppm < 0.2f) return "Poor";
    else return "Hazardous";
}

void printSensorStatus() {
    Serial.println("\n════════════════════════════════════════");
    Serial.println("          SENSOR STATUS");
    Serial.println("════════════════════════════════════════");
    Serial.printf("  PMS5003:  %s\n", pmsActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SDP810:   %s\n", sdpActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.printf("  SFA30:    %s\n", sfaActive ? "✓ ACTIVE" : "✗ INACTIVE");
    Serial.println("════════════════════════════════════════\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║    TRIPLE SENSOR AIR QUALITY MONITOR ║");
    Serial.println("║       PMS5003 + SDP810 + SFA30       ║");
    Serial.println("╚══════════════════════════════════════╝\n");
    
    // ==================== I2C INITIALIZATION ====================
    Serial.println("[I2C] Initializing I2C bus...");
    Serial.printf("  SDA: GPIO %d, SCL: GPIO %d\n", I2C_SDA, I2C_SCL);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);  // 100kHz as per datasheet
    delay(100);
    
    // ==================== PMS5003 INITIALIZATION ====================
    Serial.println("\n[PMS5003] Initializing...");
    Serial.println("  Wiring: PMS5003 TX → ESP32 GPIO 16 (RX2)");
    Serial.println("          PMS5003 RX → ESP32 GPIO 17 (TX2)");
    
    if (pmsSensor.begin()) {
        pmsActive = true;
        Serial.println("  ✓ Sensor ready!");
    } else {
        Serial.println("  ✗ Initialization failed!");
        Serial.println("  Check: 1. Power (5V), 2. TX/RX wiring, 3. 30s warm-up");
    }
    
    // ==================== SDP810 INITIALIZATION ====================
    Serial.println("\n[SDP810] Initializing...");
    Serial.printf("  I2C Address: 0x%02X\n", SDP810_I2C_ADDRESS);
    
    if (sdpSensor.begin()) {
        sdpActive = true;
        sdpSensor.startContinuousMeasurement(SDP810::CONTINUOUS_MEASUREMENT);
        Serial.println("  ✓ Continuous measurement started");
    } else {
        Serial.println("  ✗ Sensor not found!");
        Serial.println("  Check: 1. Power (3.3V), 2. I2C wiring, 3. Address jumper");
    }
    
    // ==================== SFA30 INITIALIZATION ====================
    Serial.println("\n[SFA30] Initializing...");
    Serial.println("  Wiring: SFA30 SEL pin → GND (for I2C mode)");
    Serial.println("          SFA30 ADDR pin → 3.3V (for address 0x5D)");
    Serial.printf("  I2C Address: 0x%02X\n", SFA30_I2C_ADDRESS);
    
    // Initialize sensor
    sfaSensor.begin(Wire, SFA30_I2C_ADDRESS);
    delay(100);
    
    // 1. Reset sensor
    Serial.print("  1. Resetting sensor... ");
    error = sfaSensor.deviceReset();
    if (error != NO_ERROR) {
        Serial.print("ERROR: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        Serial.println("  Check SEL pin (must be GND for I2C)");
    } else {
        Serial.println("OK");
    }
    
    delay(1000);  // Wait after reset
    
    // 2. Get device marking
    Serial.print("  2. Reading device marking... ");
    int8_t deviceMarking[32] = {0};
    error = sfaSensor.getDeviceMarking(deviceMarking, 32);
    if (error != NO_ERROR) {
        Serial.print("ERROR: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
    } else {
        sfaActive = true;
        Serial.print("Found: ");
        Serial.println((const char*)deviceMarking);
    }
    
    // 3. Start continuous measurement
    if (sfaActive) {
        Serial.print("  3. Starting continuous measurement... ");
        error = sfaSensor.startContinuousMeasurement();
        if (error != NO_ERROR) {
            Serial.print("ERROR: ");
            errorToString(error, errorMessage, sizeof(errorMessage));
            Serial.println(errorMessage);
            sfaActive = false;
        } else {
            Serial.println("OK");
            Serial.println("  Waiting 2 seconds for stabilization...");
            delay(2000);
        }
    }
    
    // ==================== SUMMARY ====================
    printSensorStatus();
    
    Serial.println("──────────────────────────────────────");
    Serial.println("Starting measurements in 3 seconds...");
    delay(3000);
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
    
    // Particle counts (optional)
    if (data.particles_03um > 0) {
        Serial.println("├────────── Particle Counts ───────────┤");
        Serial.printf("│ >0.3μm: %6d /0.1L             │\n", data.particles_03um);
        Serial.printf("│ >0.5μm: %6d /0.1L             │\n", data.particles_05um);
        Serial.printf("│ >1.0μm: %6d /0.1L             │\n", data.particles_10um);
    }
    Serial.println("└──────────────────────────────────────┘");
}

void printSDPData(const SDP810::Data& data) {
    Serial.println("┌────────────── AIR FLOW ───────────────┐");
    Serial.printf("│ Pressure: %+7.2f Pa                 │\n", data.differential_pressure);
    Serial.printf("│ Temp:     %7.2f °C                 │\n", data.temperature);
    Serial.printf("│ Flow:     %7.4f m³/s              │\n", data.air_flow);
    Serial.printf("│ Velocity: %7.2f m/s                │\n", data.air_velocity);
    
    // Interpretation
    Serial.println("├──────────── Interpretation ───────────┤");
    if (abs(data.differential_pressure) < 0.5) {
        Serial.println("│ Status:   No significant air flow    │");
    } else if (data.differential_pressure > 0) {
        Serial.printf("│ Direction: Forward (+)%16s│\n", "");
    } else {
        Serial.printf("│ Direction: Reverse (-)%16s│\n", "");
    }
    Serial.println("└──────────────────────────────────────┘");
}

void printSFAData(float hcho, float humidity, float temperature) {
    float hcho_ppm = hcho / 1000.0; // Convert ppb to ppm
    String quality = getHCHOQualityLabel(hcho);
    
    Serial.println("┌──────────── FORMALDEHYDE ─────────────┐");
    Serial.printf("│ HCHO:      %6.1f ppb                │\n", hcho);
    Serial.printf("│           (%6.3f ppm)             │\n", hcho_ppm);
    Serial.printf("│ RH:        %6.1f %%                   │\n", humidity);
    Serial.printf("│ Temp:      %6.1f °C                  │\n", temperature);
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Quality:   %-28s│\n", quality.c_str());
    
    // WHO Guidelines reference
    Serial.println("├────────── WHO Guidelines ────────────┤");
    Serial.println("│ < 80 ppb  : Good (0-0.08 ppm)       │");
    Serial.println("│ 80-100 ppb: Moderate (0.08-0.1 ppm) │");
    Serial.println("│ >100 ppb  : Poor (>0.1 ppm)         │");
    Serial.println("└──────────────────────────────────────┘");
}

void printCombinedAirQuality(const PMS5003::Data& pmsData, float hcho_ppb) {
    int pm25_aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
    String pm25_category = pmsSensor.getAQICategory(pm25_aqi);
    String hcho_quality = getHCHOQualityLabel(hcho_ppb);
    
    // Determine overall rating
    String overall = "Moderate";
    float hcho_ppm = hcho_ppb / 1000.0;
    
    if (pm25_aqi <= 50 && hcho_ppm < 0.08) {
        overall = "Good";
    } else if (pm25_aqi > 150 || hcho_ppm >= 0.2) {
        overall = "Poor";
    } else if (pm25_aqi > 100 || hcho_ppm >= 0.1) {
        overall = "Unhealthy for Sensitive";
    }
    
    Serial.println("┌───────── OVERALL AIR QUALITY ─────────┐");
    Serial.printf("│ PM2.5 AQI: %3d (%s)    │\n", pm25_aqi, pm25_category.c_str());
    Serial.printf("│ HCHO:      %6.1f ppb (%s)  │\n", hcho_ppb, hcho_quality.c_str());
    Serial.println("├──────────────────────────────────────┤");
    Serial.printf("│ Overall:   %-27s│\n", overall.c_str());
    Serial.println("└──────────────────────────────────────┘");
}

void loop() {
    static PMS5003::Data pmsData;
    static SDP810::Data sdpData;
    static float sfa_hcho = 0, sfa_humidity = 0, sfa_temp = 0;
    
    static bool pmsOk = false;
    static bool sdpOk = false;
    static bool sfaOk = false;
    
    // ==================== READ SENSORS ====================
    
    // Read PMS5003
    if (pmsActive) {
        pmsOk = pmsSensor.readData(pmsData);
    }
    
    // Read SDP810
    if (sdpActive) {
        sdpOk = sdpSensor.readMeasurement(sdpData);
    }
    
    // Read SFA30
    if (sfaActive) {
        error = sfaSensor.readMeasuredValues(sfa_hcho, sfa_humidity, sfa_temp);
        sfaOk = (error == NO_ERROR);
        if (!sfaOk) {
            errorToString(error, errorMessage, sizeof(errorMessage));
        }
    }
    
    // ==================== DISPLAY DATA ====================
    if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        
        Serial.println("\n════════════════════════════════════════");
        Serial.printf("        Reading #%d - %02d:%02d:%02d\n", 
                      lastDisplayTime / DISPLAY_INTERVAL,
                      (millis()/3600000)%24, 
                      (millis()/60000)%60, 
                      (millis()/1000)%60);
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
        
        // Display SFA30 data
        if (sfaOk) {
            printSFAData(sfa_hcho, sfa_humidity, sfa_temp);
            sfaOk = false;
        } else if (sfaActive) {
            Serial.println("[SFA30] No new data");
            if (error != NO_ERROR) {
                Serial.printf("  Error: %s\n", errorMessage);
            }
        } else {
            Serial.println("[SFA30] Sensor inactive");
        }
        
        Serial.println();
        
        // Display combined air quality if both PM and HCHO sensors are active
        if (pmsActive && sfaActive && pmsOk && sfaOk) {
            printCombinedAirQuality(pmsData, sfa_hcho);
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