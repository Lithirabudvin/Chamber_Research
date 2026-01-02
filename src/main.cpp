/*
 * SFA30 I2C Test for ESP32
 * Based on Sensirion library example with ESP32-specific modifications
 */

#include <Arduino.h>
#include <SensirionI2cSfa3x.h>
#include <Wire.h>

// ESP32 I2C pins (default)
#define I2C_SDA 21
#define I2C_SCL 22

// Macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cSfa3x sensor;

static char errorMessage[64];
static int16_t error;

void setup() {
    Serial.begin(115200);
    delay(3000);  // Wait for Serial Monitor
    
    Serial.println("=== SFA30 I2C Test ===");
    Serial.println("CONNECTIONS:");
    Serial.println("SFA30 Pin 1 (VDD)  → ESP32 3.3V");
    Serial.println("SFA30 Pin 2 (GND)  → ESP32 GND");
    Serial.println("SFA30 Pin 3 (SDA)  → ESP32 GPIO 21 (SDA)");
    Serial.println("SFA30 Pin 4 (SCL)  → ESP32 GPIO 22 (SCL)");
    Serial.println("SFA30 Pin 5 (SEL)  → GND (for I2C mode)");
    Serial.println("SFA30 Pins 6,7     → NOT CONNECTED");
    Serial.println();
    
    // Initialize I2C with ESP32 pins
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);  // 100kHz as per datasheet
    
    // Initialize sensor
    sensor.begin(Wire, SFA3X_I2C_ADDR_5D);
    delay(100);
    
    // 1. Reset sensor
    Serial.println("1. Resetting sensor...");
    error = sensor.deviceReset();
    if (error != NO_ERROR) {
        Serial.print("Reset error: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        Serial.println("Check SEL pin (must be GND for I2C)");
    } else {
        Serial.println("Reset OK");
    }
    
    delay(1000);  // Wait after reset
    
    // 2. Get device marking
    Serial.println("\n2. Reading device marking...");
    int8_t deviceMarking[32] = {0};
    error = sensor.getDeviceMarking(deviceMarking, 32);
    if (error != NO_ERROR) {
        Serial.print("Device marking error: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        Serial.println("Check: 1. Power (3.3V), 2. SEL pin (GND), 3. SDA/SCL wiring");
    } else {
        Serial.print("Device found: ");
        Serial.println((const char*)deviceMarking);
    }
    
    // 3. Start continuous measurement
    Serial.println("\n3. Starting continuous measurement...");
    error = sensor.startContinuousMeasurement();
    if (error != NO_ERROR) {
        Serial.print("Start measurement error: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        return;
    } else {
        Serial.println("Measurement started - waiting for stabilization...");
    }
    
    // Wait for sensor to stabilize (10s for HCHO to be valid)
    delay(2000);
    Serial.println("\n=== Starting readings ===\n");
}

void loop() {
    float hcho = 0.0;
    float humidity = 0.0;
    float temperature = 0.0;
    
    // Read at 4-8 second intervals based on UART testing results
    delay(8000);  // Start with 8 seconds to avoid timeout issues
    
    Serial.print("Reading sensor... ");
    
    error = sensor.readMeasuredValues(hcho, humidity, temperature);
    
    if (error != NO_ERROR) {
        Serial.print("Error: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
    } else {
        Serial.print("HCHO: ");
        Serial.print(hcho);
        Serial.print(" ppb, RH: ");
        Serial.print(humidity);
        Serial.print("%, Temp: ");
        Serial.print(temperature);
        Serial.println("°C");
    }
}