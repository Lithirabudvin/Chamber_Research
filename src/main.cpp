#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

SensirionI2cScd4x scd40;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║   SCD40 FORCED CALIBRATION TO 400   ║");
    Serial.println("╚══════════════════════════════════════╝\n");
    
    Serial.println("IMPORTANT: This assumes current CO₂ is ~400 ppm");
    Serial.println("If you're indoors with people, readings will be wrong!");
    Serial.println();
    Serial.println("Are you in FRESH OUTDOOR AIR? (y/n): ");
    
    while(!Serial.available());
    char response = Serial.read();
    
    if (response != 'y' && response != 'Y') {
        Serial.println("Aborting! Only calibrate in fresh outdoor air.");
        while(1);
    }
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    
    scd40.begin(Wire, 0x62);
    delay(100);
    
    // Stop measurements
    Serial.println("\n1. Stopping measurements...");
    scd40.stopPeriodicMeasurement();
    delay(500);
    
    // Perform forced recalibration
    Serial.println("2. Performing forced recalibration to 400 ppm...");
    
    uint16_t frcCorrection;
    int16_t error = scd40.performForcedRecalibration(400, frcCorrection);
    
    if (error == 0) {
        Serial.println("✅ Calibration successful!");
        Serial.printf("Correction applied: %d ppm\n", frcCorrection);
        
        // The correction value indicates how far off the sensor was
        // A positive value means sensor was reading too low
        // A negative value means sensor was reading too high
        Serial.printf("Your sensor was off by: %d ppm\n", frcCorrection - 400);
    } else {
        Serial.print("❌ Calibration failed. Error: ");
        char errorMessage[256];
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
    }
    
    // Re-enable auto-calibration
    Serial.println("\n3. Enabling automatic self-calibration...");
    scd40.setAutomaticSelfCalibrationEnabled(true);
    
    // Restart measurements
    Serial.println("4. Restarting measurements...");
    scd40.startPeriodicMeasurement();
    
    Serial.println("\n════════════════════════════════════════");
    Serial.println("Calibration complete!");
    Serial.println("Wait 5 minutes for readings to stabilize.");
    Serial.println("════════════════════════════════════════\n");
    
    Serial.println("Starting readings in 10 seconds...");
    for(int i = 10; i > 0; i--) {
        Serial.printf("%d... ", i);
        delay(1000);
    }
    Serial.println("\n");
}

void loop() {
    bool ready = false;
    scd40.getDataReadyStatus(ready);
    
    if (ready) {
        uint16_t co2 = 0;
        float temp = 0, hum = 0;
        int16_t error = scd40.readMeasurement(co2, temp, hum);
        
        if (error == 0) {
            Serial.println("┌────────────── CO₂ SENSOR ──────────────┐");
            Serial.printf("│ CO₂:         %5d ppm               │\n", co2);
            Serial.printf("│ Temperature:   %5.1f °C              │\n", temp);
            Serial.printf("│ Humidity:      %5.1f %%               │\n", hum);
            Serial.println("├──────────────────────────────────────┤");
            
            Serial.print("│ Quality:       ");
            if (co2 < 800) Serial.println("Excellent             │");
            else if (co2 < 1000) Serial.println("Good                  │");
            else if (co2 < 1200) Serial.println("Moderate              │");
            else if (co2 < 1500) Serial.println("Poor                  │");
            else Serial.println("Unhealthy              │");
            
            Serial.println("└──────────────────────────────────────┘\n");
        }
    } else {
        Serial.print(".");
    }
    
    delay(1000);
}