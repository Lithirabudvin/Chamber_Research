#include "PMS5003.h"

namespace PMS5003 {

    Sensor::Sensor(HardwareSerial& serial, int rxPin, int txPin) 
        : _serial(serial), _rxPin(rxPin), _txPin(txPin), 
          _active(false), _lastRead(0), _bufferIndex(0) {
    }

    Sensor::~Sensor() {
        if (_active) {
            sleep();
        }
    }

    bool Sensor::begin() {
        _serial.begin(9600, SERIAL_8N1, _rxPin, _txPin);
        delay(100);
        _active = true;
        _lastRead = 0;
        wakeUp();
        delay(30);
        return true;
    }

    bool Sensor::readData(Data& data, uint32_t timeout) {
        if (!_active) {
            data.valid = false;
            return false;
        }
        
        unsigned long startTime = millis();
        _bufferIndex = 0;
        
        while (millis() - startTime < timeout) {
            if (_serial.available()) {
                uint8_t byte = _serial.read();
                
                if (_bufferIndex == 0 && byte != 0x42) continue;
                if (_bufferIndex == 1 && byte != 0x4D) {
                    _bufferIndex = 0;
                    continue;
                }
                
                if (_bufferIndex < 32) {
                    _buffer[_bufferIndex++] = byte;
                }
                
                if (_bufferIndex == 32) {
                    if (_parseData(data, _buffer)) {
                        _lastRead = millis();
                        data.timestamp = _lastRead;
                        return true;
                    }
                    data.valid = false;
                    _bufferIndex = 0;
                }
            }
        }
        
        data.valid = false;
        return false;
    }

    bool Sensor::readDataNonBlocking(Data& data) {
        if (!_active) {
            data.valid = false;
            return false;
        }
        
        while (_serial.available()) {
            uint8_t byte = _serial.read();
            
            if (_bufferIndex == 0 && byte != 0x42) continue;
            if (_bufferIndex == 1 && byte != 0x4D) {
                _bufferIndex = 0;
                continue;
            }
            
            if (_bufferIndex < 32) {
                _buffer[_bufferIndex++] = byte;
            }
            
            if (_bufferIndex == 32) {
                if (_parseData(data, _buffer)) {
                    _lastRead = millis();
                    data.timestamp = _lastRead;
                    _bufferIndex = 0;
                    return true;
                }
                data.valid = false;
                _bufferIndex = 0;
            }
        }
        
        return false;
    }

    void Sensor::wakeUp() {
        static const uint8_t wakeCmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
        _serial.write(wakeCmd, sizeof(wakeCmd));
        delay(10);
    }

    void Sensor::sleep() {
        static const uint8_t sleepCmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
        _serial.write(sleepCmd, sizeof(sleepCmd));
        delay(10);
    }

    bool Sensor::isDataFresh(uint32_t maxAge) const {
        if (_lastRead == 0) return false;
        unsigned long age = millis() - _lastRead;
        return age < maxAge;
    }

    unsigned long Sensor::getTimeSinceLastRead() const {
        if (_lastRead == 0) return ULONG_MAX;
        return millis() - _lastRead;
    }

    bool Sensor::isSensorResponding() const {
        return _active && _lastRead != 0 && isDataFresh(10000);
    }

    bool Sensor::_parseData(Data& data, const uint8_t* buffer) {
        uint16_t checksum = _calculateChecksum(buffer, 30);
        uint16_t receivedChecksum = (buffer[30] << 8) | buffer[31];
        
        if (checksum != receivedChecksum) {
            data.valid = false;
            return false;
        }
        
        data.pm10_standard = (buffer[4] << 8) | buffer[5];
        data.pm25_standard = (buffer[6] << 8) | buffer[7];
        data.pm100_standard = (buffer[8] << 8) | buffer[9];
        data.pm10_env = (buffer[10] << 8) | buffer[11];
        data.pm25_env = (buffer[12] << 8) | buffer[13];
        data.pm100_env = (buffer[14] << 8) | buffer[15];
        data.particles_03um = (buffer[16] << 8) | buffer[17];
        data.particles_05um = (buffer[18] << 8) | buffer[19];
        data.particles_10um = (buffer[20] << 8) | buffer[21];
        data.particles_25um = (buffer[22] << 8) | buffer[23];
        data.particles_50um = (buffer[24] << 8) | buffer[25];
        data.particles_100um = (buffer[26] << 8) | buffer[27];
        data.valid = true;
        
        return true;
    }

    uint16_t Sensor::_calculateChecksum(const uint8_t* data, size_t length) {
        uint16_t sum = 0;
        for (size_t i = 0; i < length; i++) {
            sum += data[i];
        }
        return sum;
    }
}