#include "PMS5003.h"

PMS5003::PMS5003(HardwareSerial& serial, int rxPin, int txPin) 
    : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

bool PMS5003::begin() {
    _serial.begin(9600, SERIAL_8N1, _rxPin, _txPin);
    
    // Clear buffer
    while (_serial.available()) {
        _serial.read();
    }
    
    // Wake up sensor
    wakeUp();
    delay(1000);
    
    return true;
}

bool PMS5003::readData(Data& data) {
    if (!_serial.available()) return false;
    
    // Look for start bytes (0x42, 0x4D)
    uint8_t buffer[32];
    int index = 0;
    
    // Wait for start bytes
    while (_serial.available() && index < 32) {
        uint8_t b = _serial.read();
        
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
            }
            index = 0; // Reset if checksum fails
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