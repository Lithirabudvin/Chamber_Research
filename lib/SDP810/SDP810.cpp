#include "SDP810.h"
#include <math.h>

namespace SDP810 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _continuousMode(false) {
    }

    // FIXED: Correct soft reset using general call address 0x00
    bool Sensor::softReset() {
        // 1. First try to stop measurement if it's running
        if (_continuousMode) {
            stopContinuousMeasurement();
            delay(1);
        }
        
        // 2. Send soft reset using GENERAL CALL address (0x00)
        // According to datasheet page 8, section 6.3.4
        _wire.beginTransmission(0x00);  // GENERAL CALL ADDRESS, not sensor address!
        _wire.write(0x06);              // Soft reset command (8-bit)
        uint8_t error = _wire.endTransmission();
        
        _continuousMode = false;
        
        // 3. Wait for reset to complete (datasheet says max 2ms, page 3)
        delay(3);  // Slightly longer than max 2ms for safety
        
        // 4. Try to communicate with sensor to verify it's alive
        // Wait up to 50ms for sensor to recover
        for (int i = 0; i < 5; i++) {
            if (_tryCommunication()) {
                // Sensor responded!
                delay(5);  // Small additional delay
                return true;
            }
            delay(10);  // Wait 10ms before retry
        }
        
        return error == 0;
    }

    // Helper function to test communication
    bool Sensor::_tryCommunication() {
        _wire.beginTransmission(_address);
        return _wire.endTransmission() == 0;
    }

    bool Sensor::begin() {
        return _tryCommunication();
    }

    bool Sensor::readSimple(float& pressure, float& temperature) {
        if (!_continuousMode) {
            if (!startContinuousMeasurement()) {
                return false;
            }
            delay(10);
        }
        
        Data data;
        if (!readMeasurement(data)) {
            return false;
        }
        
        pressure = data.differential_pressure;
        temperature = data.temperature;
        return true;
    }

    // UPDATED: Added proper delay after starting measurement
    bool Sensor::startContinuousMeasurement(MeasurementMode mode) {
        _wire.beginTransmission(_address);
        _wire.write((mode >> 8) & 0xFF);
        _wire.write(mode & 0xFF);
        if (_wire.endTransmission() != 0) return false;
        
        _continuousMode = true;
        
        // CRITICAL: Wait for first measurement (datasheet says 8ms, page 7)
        delay(10);  // Wait 10ms for first measurement to be ready
        
        return true;
    }

    bool Sensor::stopContinuousMeasurement() {
        _wire.beginTransmission(_address);
        _wire.write(0x3F);
        _wire.write(0xF9);
        if (_wire.endTransmission() != 0) return false;
        
        _continuousMode = false;
        return true;
    }

    bool Sensor::readMeasurement(Data& data) {
        if (!_continuousMode) return false;
        
        int16_t diff_pressure_raw, temperature_raw;
        if (!_readRawValues(diff_pressure_raw, temperature_raw)) {
            return false;
        }
        
        data.differential_pressure = diff_pressure_raw / 60.0f;
        data.temperature = temperature_raw / 200.0f;
        data.air_flow = _calculateAirFlow(data.differential_pressure);
        data.air_velocity = _calculateAirVelocity(data.differential_pressure);
        data.valid = true;
        
        return true;
    }

    bool Sensor::_readRawValues(int16_t& differential_pressure_raw, int16_t& temperature_raw) {
        _wire.requestFrom(_address, (uint8_t)9);
        
        if (_wire.available() < 9) return false;
        
        uint8_t data[9];
        for (int i = 0; i < 9; i++) {
            data[i] = _wire.read();
        }
        
        if (_crc8(&data[0], 2) != data[2]) return false;
        if (_crc8(&data[3], 2) != data[5]) return false;
        
        differential_pressure_raw = (data[0] << 8) | data[1];
        temperature_raw = (data[3] << 8) | data[4];
        
        return true;
    }

    float Sensor::_calculateAirFlow(float differential_pressure) {
        const float density = 1.225f;
        const float area = 0.001f;
        
        if (differential_pressure >= 0) {
            return sqrt((2 * fabs(differential_pressure)) / density) * area;
        }
        return -sqrt((2 * fabs(differential_pressure)) / density) * area;
    }

    float Sensor::_calculateAirVelocity(float differential_pressure) {
        const float density = 1.225f;
        
        if (differential_pressure >= 0) {
            return sqrt((2 * fabs(differential_pressure)) / density);
        }
        return -sqrt((2 * fabs(differential_pressure)) / density);
    }

    uint8_t Sensor::_crc8(const uint8_t* data, size_t length) {
        uint8_t crc = 0xFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (uint8_t bit = 8; bit > 0; --bit) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc = (crc << 1);
                }
            }
        }
        return crc;
    }
}