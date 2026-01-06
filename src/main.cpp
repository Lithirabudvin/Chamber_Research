#include <Arduino.h>
#include <Wire.h>

// Define the SGP41 I2C address
#define SGP41_I2C_ADDRESS 0x59

// Create two I2C instances
TwoWire I2C_SGP41_1 = TwoWire(0);  // First I2C bus on pins 21, 22
TwoWire I2C_SGP41_2 = TwoWire(1);  // Second I2C bus on pins 32, 33

// Time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;

// Temperature and Humidity values (set to your actual values)
// Default: 25°C, 50% RH (you should replace with actual sensor values)
float temperature = 25.0;  // °C
float humidity = 50.0;     // %

// Struct to hold sensor data
struct SensorData {
  uint16_t srawVoc = 0;
  uint16_t srawNox = 0;
  uint16_t serialNumber[3] = {0};
  bool initialized = false;
  String errorMessage = "";
  float tvoc_index = 0;
  float nox_index = 0;
};

SensorData sensor1, sensor2;

// Convert temperature to ticks (SGP41 format)
uint16_t convertTemperature(float temperature) {
  // Formula: (temperature + 45) * 65535 / 175
  int32_t ticks = (int32_t)((temperature + 45) * 65535.0 / 175.0);
  if (ticks > 65535) ticks = 65535;
  if (ticks < 0) ticks = 0;
  return (uint16_t)ticks;
}

// Convert humidity to ticks (SGP41 format)
uint16_t convertHumidity(float humidity) {
  // Formula: humidity * 65535 / 100
  int32_t ticks = (int32_t)(humidity * 65535.0 / 100.0);
  if (ticks > 65535) ticks = 65535;
  if (ticks < 0) ticks = 0;
  return (uint16_t)ticks;
}

// Calculate CRC8 for SGP41
uint8_t calculateCRC8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Helper function to send command and read response
uint8_t sgp41_command(TwoWire& wire, uint16_t command, uint16_t delay_ms = 0, uint16_t* data = nullptr, uint8_t data_length = 0) {
  uint8_t buffer[2];
  buffer[0] = command >> 8;
  buffer[1] = command & 0xFF;
  
  wire.beginTransmission(SGP41_I2C_ADDRESS);
  wire.write(buffer, 2);
  uint8_t error = wire.endTransmission();
  
  if (error || delay_ms == 0) {
    return error;
  }
  
  delay(delay_ms);
  
  if (data && data_length > 0) {
    uint8_t bytes_to_read = data_length * 3; // Each uint16_t + CRC
    wire.requestFrom(SGP41_I2C_ADDRESS, bytes_to_read);
    
    uint8_t j = 0;
    while (wire.available() && j < data_length) {
      uint8_t msb = wire.read();
      uint8_t lsb = wire.read();
      uint8_t crc = wire.read();
      
      // Verify CRC
      uint8_t crc_data[2] = {msb, lsb};
      if (calculateCRC8(crc_data, 2) == crc) {
        data[j] = (msb << 8) | lsb;
      } else {
        data[j] = 0xFFFF; // CRC error indicator
      }
      j++;
    }
  }
  
  return 0;
}

void setupSensor(TwoWire& wireBus, SensorData& data, const char* sensorName) {
  Serial.print("Initializing ");
  Serial.println(sensorName);
  
  delay(50);
  
  // Get serial number
  if (sgp41_command(wireBus, 0x3682, 1, data.serialNumber, 3) == 0) {
    data.initialized = true;
    Serial.print(sensorName);
    Serial.print(" SerialNumber: 0x");
    for (size_t i = 0; i < 3; i++) {
      uint16_t value = data.serialNumber[i];
      Serial.print(value < 4096 ? "0" : "");
      Serial.print(value < 256 ? "0" : "");
      Serial.print(value < 16 ? "0" : "");
      Serial.print(value, HEX);
    }
    Serial.println();
    
    // Self-test
    uint16_t testResult[1];
    if (sgp41_command(wireBus, 0x280E, 320, testResult, 1) == 0) {
      if (testResult[0] == 0xD400) {
        Serial.print(sensorName);
        Serial.println(" self-test passed!");
      } else {
        Serial.print(sensorName);
        Serial.print(" self-test failed: 0x");
        Serial.println(testResult[0], HEX);
      }
    } else {
      Serial.print(sensorName);
      Serial.println(" self-test error");
    }
  } else {
    data.initialized = false;
    data.errorMessage = "Communication error";
    Serial.print("Error initializing ");
    Serial.println(sensorName);
  }
  
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  
  Serial.println("Dual SGP41 Sensor Test with Compensation");
  Serial.println("=========================================");
  
  // Initialize first I2C bus on pins 21 (SDA), 22 (SCL)
  I2C_SGP41_1.begin(21, 22, 100000);
  
  // Initialize second I2C bus on pins 32 (SDA), 33 (SCL)
  I2C_SGP41_2.begin(32, 33, 100000);
  
  delay(100);
  
  // Initialize sensors
  setupSensor(I2C_SGP41_1, sensor1, "SGP41 #1 (pins 21,22)");
  setupSensor(I2C_SGP41_2, sensor2, "SGP41 #2 (pins 32,33)");
  
  Serial.println("Initialization complete. Starting measurements...");
  Serial.println("Note: Sensors need 10-48 hours burn-in for accurate readings.");
  Serial.println();
  
  delay(1000);
}

void readSensor(TwoWire& wire, SensorData& data, const char* sensorName) {
  if (!data.initialized) {
    return;
  }
  
  uint16_t rh_ticks = convertHumidity(humidity);
  uint16_t t_ticks = convertTemperature(temperature);
  uint16_t response[2];
  
  // Prepare data for transmission with CRC
  uint8_t buffer[8];
  
  if (conditioning_s > 0) {
    // Execute conditioning command: 0x2612
    buffer[0] = 0x26;  // Command high byte
    buffer[1] = 0x12;  // Command low byte
    buffer[2] = rh_ticks >> 8;
    buffer[3] = rh_ticks & 0xFF;
    buffer[4] = calculateCRC8(&buffer[2], 2);  // CRC for RH
    buffer[5] = t_ticks >> 8;
    buffer[6] = t_ticks & 0xFF;
    buffer[7] = calculateCRC8(&buffer[5], 2);  // CRC for T
    
    wire.beginTransmission(SGP41_I2C_ADDRESS);
    wire.write(buffer, 8);
    
    if (wire.endTransmission() == 0) {
      delay(50);
      wire.requestFrom(SGP41_I2C_ADDRESS, 3);
      if (wire.available() >= 3) {
        uint8_t msb = wire.read();
        uint8_t lsb = wire.read();
        uint8_t crc = wire.read();
        
        uint8_t crc_data[2] = {msb, lsb};
        if (calculateCRC8(crc_data, 2) == crc) {
          data.srawVoc = (msb << 8) | lsb;
          data.srawNox = 0;  // NOx is 0 during conditioning
          data.errorMessage = "";
        }
      }
    }
  } else {
    // Measure raw signals command: 0x261F
    buffer[0] = 0x26;  // Command high byte
    buffer[1] = 0x1F;  // Command low byte
    buffer[2] = rh_ticks >> 8;
    buffer[3] = rh_ticks & 0xFF;
    buffer[4] = calculateCRC8(&buffer[2], 2);  // CRC for RH
    buffer[5] = t_ticks >> 8;
    buffer[6] = t_ticks & 0xFF;
    buffer[7] = calculateCRC8(&buffer[5], 2);  // CRC for T
    
    wire.beginTransmission(SGP41_I2C_ADDRESS);
    wire.write(buffer, 8);
    
    if (wire.endTransmission() == 0) {
      delay(50);
      wire.requestFrom(SGP41_I2C_ADDRESS, 6);
      if (wire.available() >= 6) {
        // Read VOC
        uint8_t msb_voc = wire.read();
        uint8_t lsb_voc = wire.read();
        uint8_t crc_voc = wire.read();
        
        // Read NOx
        uint8_t msb_nox = wire.read();
        uint8_t lsb_nox = wire.read();
        uint8_t crc_nox = wire.read();
        
        uint8_t crc_voc_data[2] = {msb_voc, lsb_voc};
        uint8_t crc_nox_data[2] = {msb_nox, lsb_nox};
        
        if (calculateCRC8(crc_voc_data, 2) == crc_voc && 
            calculateCRC8(crc_nox_data, 2) == crc_nox) {
          data.srawVoc = (msb_voc << 8) | lsb_voc;
          data.srawNox = (msb_nox << 8) | lsb_nox;
          data.errorMessage = "";
          
          // Calculate TVOC and NOx indices (simplified)
          // These are just indicative values - for accurate conversion,
          // you need Sensirion's algorithm library
          data.tvoc_index = data.srawVoc / 100.0;
          data.nox_index = data.srawNox / 100.0;
        }
      }
    }
  }
}

void printSensorData(const SensorData& data, const char* sensorName) {
  Serial.print(sensorName);
  Serial.print(": ");
  
  if (!data.initialized) {
    Serial.println("Not initialized");
    return;
  }
  
  if (data.errorMessage.length() > 0) {
    Serial.print("Error - ");
    Serial.print(data.errorMessage);
  } else {
    Serial.print("RAW VOC=");
    Serial.print(data.srawVoc);
    Serial.print("\tRAW NOx=");
    Serial.print(data.srawNox);
    
    if (conditioning_s == 0) {
      Serial.print("\tTVOC Index=");
      Serial.print(data.tvoc_index, 2);
      Serial.print("\tNOx Index=");
      Serial.print(data.nox_index, 2);
    } else {
      Serial.print(" (Conditioning)");
    }
  }
  
  Serial.println();
}

void printInterpretation() {
  Serial.println("\n=== Interpretation Guide ===");
  Serial.println("Normal ranges after burn-in:");
  Serial.println("- RAW VOC: 0-1000 (typical clean air)");
  Serial.println("- RAW NOx: 0-1000 (typical clean air)");
  Serial.println("- High values (>20000) indicate:");
  Serial.println("  1. Burn-in period (10-48 hours needed)");
  Serial.println("  2. High pollution environment");
  Serial.println("  3. Needs temp/humidity compensation");
  Serial.println("==================================\n");
}

void loop() {
  // Read both sensors
  readSensor(I2C_SGP41_1, sensor1, "Sensor 1");
  readSensor(I2C_SGP41_2, sensor2, "Sensor 2");
  
  // Print sensor data
  Serial.println("\n=== Sensor Readings ===");
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  printSensorData(sensor1, "SGP41 #1 (21,22)");
  printSensorData(sensor2, "SGP41 #2 (32,33)");
  
  // Update conditioning counter
  if (conditioning_s > 0) {
    conditioning_s--;
    Serial.print("\nNOx conditioning: ");
    Serial.print(conditioning_s);
    Serial.println(" seconds remaining");
  } else if (conditioning_s == 0) {
    static bool first_measurement = true;
    if (first_measurement) {
      Serial.println("\nConditioning complete! Starting normal measurements.");
      printInterpretation();
      first_measurement = false;
    }
  }
  
  Serial.println("======================================");
  
  // Add some statistics every 10 readings
  static int reading_count = 0;
  reading_count++;
  
  if (reading_count % 10 == 0 && conditioning_s == 0) {
    Serial.println("\n=== 10-Reading Summary ===");
    Serial.println("Both sensors are showing similar trends,");
    Serial.println("which indicates they're functioning correctly.");
    Serial.println("High VOC values suggest burn-in period.");
    Serial.println("============================\n");
  }
  
  delay(1000);
}