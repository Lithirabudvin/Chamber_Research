#include <Arduino.h>
#include "PMS5003.h"
#include "SDP810.h"
#include "SFA30.h"

// PMS5003 Configuration
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

// SDP810 Configuration (I2C)
#define SDP810_SDA_PIN 21
#define SDP810_SCL_PIN 22
#define SDP810_I2C_ADDRESS 0x25

// SFA30 Configuration (I2C)
#define SFA30_SDA_PIN 21  // Can share with SDP810
#define SFA30_SCL_PIN 22  // Can share with SDP810
#define SFA30_I2C_ADDRESS 0x5D

// Create sensor objects
PMS5003 pmsSensor(Serial2, PMS_RX_PIN, PMS_TX_PIN);
SDP810 sdpSensor(Wire, SDP810_I2C_ADDRESS);
SFA30 sfaSensor(Wire, SFA30_I2C_ADDRESS);

// Statistics
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 3000; // Display every 3 seconds
bool pmsActive = false;
bool sdpActive = false;
bool sfaActive = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║    TRIPLE SENSOR AIR QUALITY MONITOR ║");
  Serial.println("║   PMS5003 + SDP810 + SFA30           ║");
  Serial.println("╚══════════════════════════════════════╝\n");
  
  // Initialize I2C bus
  Serial.println("[I2C] Initializing I2C bus...");
  Wire.begin(SDP810_SDA_PIN, SDP810_SCL_PIN);
  Wire.setClock(100000); // 100kHz I2C speed
  delay(100);
  
  // Initialize PMS5003
  // Serial.println("[PMS5003] Initializing...");
  // if (pmsSensor.begin()) {
  //   pmsActive = true;
  //   Serial.println("[PMS5003] ✓ Sensor ready!");
  // } else {
  //   Serial.println("[PMS5003] ✗ Initialization failed!");
  // }
  
  // Initialize SDP810
  // Serial.println("[SDP810] Initializing...");
  // if (sdpSensor.begin()) {
  //   sdpActive = true;
  //   sdpSensor.startContinuousMeasurement(SDP810::CONTINUOUS_MEASUREMENT);
  //   Serial.println("[SDP810] ✓ Continuous measurement started");
  // } else {
  //   Serial.println("[SDP810] ✗ Sensor not found!");
  // }
  
  // Initialize SFA30
  Serial.println("[SFA30] Initializing...");
  if (sfaSensor.begin()) {
    sfaActive = true;
    
    // Get sensor info
    String serial = sfaSensor.getSerialNumberString();
    uint8_t productType = sfaSensor.getProductType();
    uint8_t productVersion = sfaSensor.getProductVersion();
    
    Serial.println("[SFA30] ✓ Sensor detected!");
    Serial.printf("  Serial: %s\n", serial.c_str());
    Serial.printf("  Type: 0x%02X, Version: 0x%02X\n", productType, productVersion);
    
    // Start continuous measurement
    if (sfaSensor.startContinuousMeasurement(SFA30::CONTINUOUS)) {
      Serial.println("[SFA30] ✓ Continuous measurement started");
    } else {
      Serial.println("[SFA30] ✗ Failed to start measurement");
      sfaActive = false;
    }
  } else {
    Serial.println("[SFA30] ✗ Sensor not found!");
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

// void printPMSData(const PMS5003::Data& pmsData) {
//   int aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
//   String category = pmsSensor.getAQICategory(aqi);
  
//   Serial.println("┌────────────── PM SENSOR ──────────────┐");
//   Serial.printf("│ PM1.0:  %3d μg/m³                     │\n", pmsData.pm10_standard);
//   Serial.printf("│ PM2.5:  %3d μg/m³                     │\n", pmsData.pm25_standard);
//   Serial.printf("│ PM10:   %3d μg/m³                     │\n", pmsData.pm100_standard);
//   Serial.println("├──────────────────────────────────────┤");
//   Serial.printf("│ AQI:    %3d (%s)          │\n", aqi, category.c_str());
//   Serial.println("└──────────────────────────────────────┘");
// }

// void printSDPData(const SDP810::Data& sdpData) {
//   Serial.println("┌────────────── AIR FLOW ───────────────┐");
//   Serial.printf("│ Pressure: %+7.2f Pa                 │\n", sdpData.differential_pressure);
//   Serial.printf("│ Temp:     %7.2f °C                 │\n", sdpData.temperature);
//   Serial.printf("│ Flow:     %7.4f m³/s              │\n", sdpData.air_flow);
//   Serial.printf("│ Velocity: %7.2f m/s                │\n", sdpData.air_velocity);
  
//   // Interpretation
//   Serial.println("├──────────── Interpretation ───────────┤");
//   if (abs(sdpData.differential_pressure) < 1.0) {
//     Serial.println("│ Status:   No significant air flow    │");
//   } else if (sdpData.differential_pressure > 0) {
//     Serial.printf("│ Direction: Forward (+)%16s│\n", "");
//   } else {
//     Serial.printf("│ Direction: Reverse (-)%16s│\n", "");
//   }
//   Serial.println("└──────────────────────────────────────┘");
// }

void printSFAData(const SFA30::Data& sfaData) {
  String quality = SFA30::getAirQualityLabel(sfaData.formaldehyde);
  
  Serial.println("┌──────────── FORMALDEHYDE ─────────────┐");
  Serial.printf("│ HCHO:    %6.3f ppm                 │\n", sfaData.formaldehyde);
  Serial.printf("│ RH:      %6.1f %%                   │\n", sfaData.humidity);
  Serial.printf("│ Temp:    %6.1f °C                  │\n", sfaData.temperature);
  Serial.println("├──────────────────────────────────────┤");
  Serial.printf("│ Quality: %-28s│\n", quality.c_str());
  
  // WHO Guidelines reference
  Serial.println("├────────── WHO Guidelines ────────────┤");
  Serial.println("│ < 0.08 ppm : Good                   │");
  Serial.println("│ 0.08-0.1 ppm: Moderate              │");
  Serial.println("│ > 0.1 ppm  : Poor                   │");
  Serial.println("└──────────────────────────────────────┘");
}

// void printCombinedAirQuality(const PMS5003::Data& pmsData, const SFA30::Data& sfaData) {
//   int pm25_aqi = pmsSensor.calculateAQI(pmsData.pm25_standard);
//   String pm25_category = pmsSensor.getAQICategory(pm25_aqi);
//   String hcho_quality = SFA30::getAirQualityLabel(sfaData.formaldehyde);
  
//   // Simple overall rating
//   String overall = "Moderate";
  
//   if (pm25_aqi <= 50 && sfaData.formaldehyde < 0.08) {
//     overall = "Good";
//   } else if (pm25_aqi > 150 || sfaData.formaldehyde >= 0.2) {
//     overall = "Poor";
//   } else if (pm25_aqi > 100 || sfaData.formaldehyde >= 0.1) {
//     overall = "Unhealthy for Sensitive";
//   }
  
//   Serial.println("┌───────── OVERALL AIR QUALITY ─────────┐");
//   Serial.printf("│ PM2.5 AQI: %3d (%s)    │\n", pm25_aqi, pm25_category.c_str());
//   Serial.printf("│ HCHO:      %6.3f ppm (%s)  │\n", sfaData.formaldehyde, hcho_quality.c_str());
//   Serial.println("├──────────────────────────────────────┤");
//   Serial.printf("│ Overall:   %-27s│\n", overall.c_str());
//   Serial.println("└──────────────────────────────────────┘");
// }

void loop() {
 // static PMS5003::Data pmsData;
 // static SDP810::Data sdpData;
  static SFA30::Data sfaData;
//  static bool pmsOk = false;
//  static bool sdpOk = false;
  static bool sfaOk = false;
  
  // Read PMS5003 data
  // if (pmsActive && pmsSensor.readData(pmsData)) {
  //   pmsOk = true;
  // }
  
  // Read SDP810 data
  // if (sdpActive && sdpSensor.readMeasurement(sdpData)) {
  //   sdpOk = true;
  // }
  
  // Read SFA30 data
  if (sfaActive && sfaSensor.readMeasurement(sfaData)) {
    sfaOk = true;
  }
  
  // Display data every DISPLAY_INTERVAL milliseconds
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
    // if (pmsOk) {
    //   printPMSData(pmsData);
    //   pmsOk = false;
    // } else if (pmsActive) {
    //   Serial.println("[PMS5003] No new data");
    // }
    
//    Serial.println();
    
    // Display SDP810 data
    // if (sdpOk) {
    //   printSDPData(sdpData);
    //   sdpOk = false;
    // } else if (sdpActive) {
    //   Serial.println("[SDP810] No new data");
    // }
    
//    Serial.println();
    
    // Display SFA30 data
    if (sfaOk) {
      printSFAData(sfaData);
      sfaOk = false;
    } else if (sfaActive) {
      Serial.println("[SFA30] No new data");
    }
    
    Serial.println();
    
    // Display combined air quality if both sensors have data
    // if (pmsActive && sfaActive) {
    //   printCombinedAirQuality(pmsData, sfaData);
    // }
    
    // Serial.println("\n──────────────────────────────────────");
    // Serial.printf("Next update in: %.1f seconds\n", DISPLAY_INTERVAL / 1000.0);
  }
  
  // Optional: Fan cleaning for SFA30 (every 10 minutes)
  static unsigned long lastFanClean = 0;
  if (sfaActive && millis() - lastFanClean > 600000) { // 10 minutes
    Serial.println("[SFA30] Starting fan cleaning...");
    if (sfaSensor.startFanCleaning()) {
      Serial.println("[SFA30] Fan cleaning started");
    }
    lastFanClean = millis();
  }
  
  delay(100); // Small delay to prevent CPU overload
}