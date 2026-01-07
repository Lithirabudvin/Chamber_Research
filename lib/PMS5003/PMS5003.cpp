#include "PMS5003.h"

PMS5003::PMS5003(HardwareSerial& serial, int rxPin, int txPin) 
    : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

bool PMS5003::begin() {
    _serial.begin(9600, SERIAL_8N1, _rxPin, _txPin);
    _serial.setTimeout(100);  // Add timeout for readBytes
    _serial.setRxBufferSize(256);  // Increase buffer
    
    // Clear buffer
    while (_serial.available()) {
        _serial.read();
    }
    
    // Wake up sensor
    wakeUp();
    delay(1000);
    
    return true;
}

// IMPROVED VERSION with timeout and better packet handling
bool PMS5003::readData(Data& data, uint32_t timeout_ms) {
    if (!_serial.available()) {
        delay(10);
        return false;
    }
    
    uint8_t buffer[32];
    int index = 0;
    unsigned long startTime = millis();
    
    // Look for start bytes with timeout
    while ((millis() - startTime) < timeout_ms && index < 32) {
        if (_serial.available()) {
            uint8_t b = _serial.read();
            
            // Look for start sequence
            if (index == 0 && b != 0x42) continue;
            if (index == 1 && b != 0x4D) {
                index = 0;
                continue;
            }
            
            buffer[index++] = b;
            
            if (index == 32) {
                // Full packet received
                if (_checksumValid(buffer)) {
                    return _parseData(buffer, data);
                } else {
                    index = 0; // Reset if checksum fails
                }
            }
        } else {
            delay(1); // Small delay to prevent busy waiting
        }
    }
    
    // If we timed out but have partial data, clear it
    if (index > 0) {
        // Serial.printf("[PMS] Timeout with %d bytes\n", index);
    }
    
    return false;
}

// Alternative: Simple non-blocking version
bool PMS5003::readDataNonBlocking(Data& data) {
    if (_serial.available() < 32) return false;
    
    uint8_t buffer[32];
    int bytesRead = _serial.readBytes(buffer, 32);
    
    if (bytesRead == 32) {
        // Check if we have valid start bytes
        if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
            if (_checksumValid(buffer)) {
                return _parseData(buffer, data);
            }
        } else {
            // Wrong start bytes, discard and try to sync
            // Look for 0x42 in the buffer
            for (int i = 1; i < 32; i++) {
                if (buffer[i] == 0x42) {
                    // Discard everything before this
                    uint8_t temp[32];
                    int remaining = 32 - i;
                    // Move remaining bytes to temp
                    for (int j = 0; j < remaining; j++) {
                        temp[j] = buffer[i + j];
                    }
                    // Read the rest
                    _serial.readBytes(&temp[remaining], i);
                    // Check this new buffer
                    if (temp[0] == 0x42 && temp[1] == 0x4D && _checksumValid(temp)) {
                        return _parseData(temp, data);
                    }
                    break;
                }
            }
        }
    }
    
    return false;
}

bool PMS5003::isAvailable() {
    return _serial.available() >= 32;
}

bool PMS5003::_parseData(uint8_t* buffer, Data& data) {
    data.pm10_standard = (buffer[2] << 8) | buffer[3];
    data.pm25_standard = (buffer[4] << 8) | buffer[5];
    data.pm100_standard = (buffer[6] << 8) | buffer[7];
    data.pm10_env = (buffer[8] << 8) | buffer[9];
    data.pm25_env = (buffer[10] << 8) | buffer[11];
    data.pm100_env = (buffer[12] << 8) | buffer[13];
    data.particles_03um = (buffer[14] << 8) | buffer[15];
    data.particles_05um = (buffer[16] << 8) | buffer[17];
    data.particles_10um = (buffer[18] << 8) | buffer[19];
    data.particles_25um = (buffer[20] << 8) | buffer[21];
    data.particles_50um = (buffer[22] << 8) | buffer[23];
    data.particles_100um = (buffer[24] << 8) | buffer[25];
    
    return true;
}

bool PMS5003::_checksumValid(uint8_t* buffer) {
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
        sum += buffer[i];
    }
    
    uint16_t checksum = (buffer[30] << 8) | buffer[31];
    return (sum == checksum);
}

// ... rest of your existing functions ...

float PMS5003::_mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int PMS5003::calculateAQI(float pm25) {
    // US EPA AQI calculation for PM2.5
    if (pm25 <= 12.0) return _mapFloat(pm25, 0, 12, 0, 50);
    else if (pm25 <= 35.4) return _mapFloat(pm25, 12.1, 35.4, 51, 100);
    else if (pm25 <= 55.4) return _mapFloat(pm25, 35.5, 55.4, 101, 150);
    else if (pm25 <= 150.4) return _mapFloat(pm25, 55.5, 150.4, 151, 200);
    else if (pm25 <= 250.4) return _mapFloat(pm25, 150.5, 250.4, 201, 300);
    else return _mapFloat(pm25, 250.5, 500.4, 301, 500);
}

String PMS5003::getAQICategory(int aqi) {
    if (aqi <= 50) return "Good";
    if (aqi <= 100) return "Moderate";
    if (aqi <= 150) return "Unhealthy for Sensitive Groups";
    if (aqi <= 200) return "Unhealthy";
    if (aqi <= 300) return "Very Unhealthy";
    return "Hazardous";
}

String PMS5003::getAQIAdvice(int aqi) {
    if (aqi <= 50) return "Air quality is satisfactory";
    if (aqi <= 100) return "Acceptable air quality";
    if (aqi <= 150) return "Sensitive groups should reduce outdoor exertion";
    if (aqi <= 200) return "Everyone may experience health effects";
    if (aqi <= 300) return "Health alert: everyone may experience serious effects";
    return "Emergency conditions";
}

void PMS5003::sleep() {
    uint8_t sleepCmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
    _serial.write(sleepCmd, sizeof(sleepCmd));
}

void PMS5003::wakeUp() {
    uint8_t wakeCmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
    _serial.write(wakeCmd, sizeof(wakeCmd));
}