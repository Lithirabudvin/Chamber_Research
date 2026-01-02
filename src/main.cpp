#include <Arduino.h>
#include "PMS5003.h"
#include "SDP810.h"
#include <SensirionI2cSfa3x.h>
#include <Wire.h>

// PMS5003 Configuration
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

// SDP810 Configuration (I2C)
#define SDP810_SDA_PIN 21
#define SDP810_SCL_PIN 22
#define SDP810_I2C_ADDRESS 0x25

// SFA30 Configuration (I2C) - shares I2C bus with SDP810
#define SFA30_I2C_ADDRESS 0x5D

// Macro for Sensirion library
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

// Create sensor objects
PMS5003 pmsSensor(Serial2, PMS_RX_PIN, PMS_TX_PIN);
SDP810 sdpSensor(Wire, SDP810_I2C_ADDRESS);
SensirionI2cSfa3x sfaSensor;  // Using official Sensirion library

// Statistics
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 3000; // Display every 3 seconds
unsigned long lastSFARead = 0;
const unsigned long SFA_READ_INTERVAL = 500; // Read SFA30 every 500ms
bool pmsActive = false;
bool sdpActive = false;
bool sfaActive = false;

// SFA30 data structure
struct SFA30Data {
  float formaldehyde;  // ppm
  float humidity;      // %RH
  float temperature;   // °C
  bool valid;
};

SFA30Data sfaData = {0, 0, 0, false};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║    TRIPLE SENSOR AIR QUALITY MONITOR ║");
  Serial.println("║   PMS5003 + SDP810 + SFA30           ║");
  Serial.println("╚══════════════════════════════════════╝\n");
  
  // Initialize I2C bus (shared by SDP810 and SFA30)
  Serial.println("[I2C] Initializing I2C bus...");
  Wire.begin(SDP810_SDA_PIN, SDP810_SCL_PIN);
  Wire.setClock(100000); // 100kHz I2C speed
  delay(100);
  
  // Initialize PMS5003
  Serial.println("[PMS5003] Initializing...");
  if (pmsSensor.begin()) {
    pmsActive = true;
    Serial.println("[PMS5003] ✓ Sensor ready!");
  } else {
    Serial.println("[PMS5003] ✗ Initialization failed!");
  }
  
  // Initialize SDP810
  Serial.println("[SDP810] Initializing...");
  if (sdpSensor.begin()) {
    sdpActive = true;
    sdpSensor.startContinuousMeasurement(SDP810::CONTINUOUS_MEASUREMENT);
    Serial.println("[SDP810] ✓ Continuous measurement started");
  } else {
    Serial.println("[SDP810] ✗ Sensor not found!");
  }
  
  // Initialize SFA30 with official Sensirion library
  Serial.println("[SFA30] Initializing...");
  sfaSensor.begin(Wire, SFA3X_I2C_ADDR_5D);
  
  char errorMessage[64];
  int16_t error;
  
  // Reset sensor
  error = sfaSensor.deviceReset();
  if (error == NO_ERROR) {
    Serial.println("[SFA30] ✓ Reset successful");
    delay(1000);
  } else {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("[SFA30] ✗ Reset failed: %s\n", errorMessage);
  }
  
  // Get device marking
  int8_t deviceMarking[32] = {0};
  error = sfaSensor.getDeviceMarking(deviceMarking, 32);
  if (error == NO_ERROR) {
    Serial.printf("[SFA30] ✓ Device: %s\n", (const char*)deviceMarking);
  } else {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("[SFA30] Device marking error: %s\n", errorMessage);
  }
  
  // Start continuous measurement
  error = sfaSensor.startContinuousMeasurement();
  if (error == NO_ERROR) {
    sfaActive = true;
    Serial.println("[SFA30] ✓ Continuous measurement started");
    Serial.println("[SFA30] Note: HCHO readings will be 0 for first 10 seconds");
  } else {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("[SFA30] ✗ Failed to start: %s\n", errorMessage);
  }
  
  Serial.println("\n──────────────────────────────────────");
  Serial.println("Sensor Status Summary:");
  Serial.printf("  PMS5003: %s\n", pmsActive ? "✓ ACTIVE" : "✗ INACTIVE");
  Serial.printf("  SDP810:  %s\n", sdpActive ? "✓ ACTIVE" : "✗ INACTIVE");
  Serial.printf("  SFA30:   %s\n", sfaActive ? "✓ ACTIVE" : "✗ INACTIVE");
  Serial.println("──────────────────────────────────────");
  Serial.println("Starting measurements in 3 seconds...");
  delay(3000);
}

void printPMSData(const PMS5003::Data& pmsData) {
  int aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
  String category = pmsSensor.getAQICategory(aqi);
  
  Serial.println("┌────────────── PM SENSOR ──────────────┐");
  Serial.printf("│ PM1.0:  %3d μg/m³                     │\n", pmsData.pm10_standard);
  Serial.printf("│ PM2.5:  %3d μg/m³                     │\n", pmsData.pm25_standard);
  Serial.printf("│ PM10:   %3d μg/m³                     │\n", pmsData.pm100_standard);
  Serial.println("├──────────────────────────────────────┤");
  Serial.printf("│ AQI:    %3d (%s)          │\n", aqi, category.c_str());
  Serial.println("└──────────────────────────────────────┘");
}

void printSDPData(const SDP810::Data& sdpData) {
  Serial.println("┌────────────── AIR FLOW ───────────────┐");
  Serial.printf("│ Pressure: %+7.2f Pa                 │\n", sdpData.differential_pressure);
  Serial.printf("│ Temp:     %7.2f °C                 │\n", sdpData.temperature);
  Serial.printf("│ Flow:     %7.4f m³/s              │\n", sdpData.air_flow);
  Serial.printf("│ Velocity: %7.2f m/s                │\n", sdpData.air_velocity);
  
  Serial.println("├──────────── Interpretation ───────────┤");
  if (abs(sdpData.differential_pressure) < 1.0) {
    Serial.println("│ Status:   No significant air flow    │");
  } else if (sdpData.differential_pressure > 0) {
    Serial.printf("│ Direction: Forward (+)%16s│\n", "");
  } else {
    Serial.printf("│ Direction: Reverse (-)%16s│\n", "");
  }
  Serial.println("└──────────────────────────────────────┘");
}

void printSFAData(const SFA30Data& data) {
  if (!data.valid) {
    Serial.println("[SFA30] No valid data");
    return;
  }
  
  // Convert ppb to ppm for display (library gives ppb)
  float hcho_ppm = data.formaldehyde / 1000.0;
  
  String quality = "Good";
  if (hcho_ppm >= 0.1) {
    quality = "Poor";
  } else if (hcho_ppm >= 0.08) {
    quality = "Moderate";
  }
  
  Serial.println("┌──────────── FORMALDEHYDE ─────────────┐");
  Serial.printf("│ HCHO:    %6.1f ppb (%6.3f ppm)     │\n", data.formaldehyde, hcho_ppm);
  Serial.printf("│ RH:      %6.1f %%                   │\n", data.humidity);
  Serial.printf("│ Temp:    %6.1f °C                  │\n", data.temperature);
  Serial.println("├──────────────────────────────────────┤");
  Serial.printf("│ Quality: %-28s│\n", quality.c_str());
  Serial.println("├────────── WHO Guidelines ────────────┤");
  Serial.println("│ < 0.08 ppm : Good                   │");
  Serial.println("│ 0.08-0.1 ppm: Moderate              │");
  Serial.println("│ > 0.1 ppm  : Poor                   │");
  Serial.println("└──────────────────────────────────────┘");
}

void printCombinedAirQuality(const PMS5003::Data& pmsData, const SFA30Data& sfaData) {
  int pm25_aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
  String pm25_category = pmsSensor.getAQICategory(pm25_aqi);
  
  float hcho_ppm = sfaData.formaldehyde / 1000.0;
  String hcho_quality = "Good";
  if (hcho_ppm >= 0.1) hcho_quality = "Poor";
  else if (hcho_ppm >= 0.08) hcho_quality = "Moderate";
  
  String overall = "Moderate";
  if (pm25_aqi <= 50 && hcho_ppm < 0.08) {
    overall = "Good";
  } else if (pm25_aqi > 150 || hcho_ppm >= 0.2) {
    overall = "Poor";
  } else if (pm25_aqi > 100 || hcho_ppm >= 0.1) {
    overall = "Unhealthy for Sensitive";
  }
  
  Serial.println("┌───────── OVERALL AIR QUALITY ─────────┐");
  Serial.printf("│ PM2.5 AQI: %3d (%s)    │\n", pm25_aqi, pm25_category.c_str());
  Serial.printf("│ HCHO:      %6.3f ppm (%s)  │\n", hcho_ppm, hcho_quality.c_str());
  Serial.println("├──────────────────────────────────────┤");
  Serial.printf("│ Overall:   %-27s│\n", overall.c_str());
  Serial.println("└──────────────────────────────────────┘");
}

void readSFA30Data() {
  if (!sfaActive) return;
  
  static char errorMessage[64];
  int16_t error;
  float hcho, humidity, temperature;
  
  // Read with retry on error
  error = sfaSensor.readMeasuredValues(hcho, humidity, temperature);
  if (error != NO_ERROR) {
    delay(50);
    error = sfaSensor.readMeasuredValues(hcho, humidity, temperature);
  }
  
  if (error == NO_ERROR) {
    sfaData.formaldehyde = hcho;  // ppb
    sfaData.humidity = humidity;
    sfaData.temperature = temperature;
    sfaData.valid = true;
  } else {
    sfaData.valid = false;
    // Only print errors occasionally to avoid spam
    static unsigned long lastErrorPrint = 0;
    if (millis() - lastErrorPrint > 5000) {
      errorToString(error, errorMessage, sizeof(errorMessage));
      Serial.printf("[SFA30] Read error: %s\n", errorMessage);
      lastErrorPrint = millis();
    }
  }
}

void loop() {
  static PMS5003::Data pmsData;
  static SDP810::Data sdpData;
  static bool pmsOk = false;
  static bool sdpOk = false;
  
  // Read SFA30 at 500ms intervals (2 Hz for optimal performance)
  if (millis() - lastSFARead >= SFA_READ_INTERVAL) {
    lastSFARead = millis();
    readSFA30Data();
  }
  
  // Read PMS5003 data
  if (pmsActive && pmsSensor.readData(pmsData)) {
    pmsOk = true;
  }
  
  // Read SDP810 data
  if (sdpActive && sdpSensor.readMeasurement(sdpData)) {
    sdpOk = true;
  }
  
  // Display data every DISPLAY_INTERVAL milliseconds
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    
    Serial.println("\n════════════════════════════════════════");
    Serial.printf("        Reading #%lu - %02d:%02d:%02d\n", 
                  lastDisplayTime / DISPLAY_INTERVAL,
                  (int)((millis()/3600000)%24), 
                  (int)((millis()/60000)%60), 
                  (int)((millis()/1000)%60));
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
    if (sfaData.valid) {
      printSFAData(sfaData);
    } else if (sfaActive) {
      Serial.println("[SFA30] No valid data yet");
    }
    
    Serial.println();
    
    // Display combined air quality if both PMS and SFA have data
    if (pmsActive && sfaData.valid) {
      printCombinedAirQuality(pmsData, sfaData);
    }
    
    Serial.println("\n──────────────────────────────────────");
    Serial.printf("Next update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
  }
  
  delay(10); // Small delay to prevent CPU overload
}