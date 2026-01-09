#include "SDP810.h"
#include <math.h>  // Add this for sqrt and abs functions

namespace SDP810 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _continuousMode(false) {
    }

    bool Sensor::begin() {
        _wire.beginTransmission(_address);
        return _wire.endTransmission() == 0;
    }

    // ADD THIS SIMPLE READ FUNCTION
    bool Sensor::readSimple(float& pressure, float& temperature) {
        // Start continuous measurement if not already started
        if (!_continuousMode) {
            if (!startContinuousMeasurement()) {
                return false;
            }
            delay(10); // Small delay after starting
        }
        
        Data data;
        if (!readMeasurement(data)) {
            return false;
        }
        
        pressure = data.differential_pressure;
        temperature = data.temperature;
        return true;
    }

    bool Sensor::startContinuousMeasurement(MeasurementMode mode) {
        _wire.beginTransmission(_address);
        _wire.write((mode >> 8) & 0xFF);
        _wire.write(mode & 0xFF);
        if (_wire.endTransmission() != 0) return false;
        
        _continuousMode = true;
        delay(10);  // Allow sensor to start measurements
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
        
        // Convert raw values to engineering units
        data.differential_pressure = diff_pressure_raw / 60.0f;  // Scale factor from datasheet
        data.temperature = temperature_raw / 200.0f;            // Scale factor from datasheet
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
        
        // Check CRC for differential pressure
        if (_crc8(&data[0], 2) != data[2]) return false;
        
        // Check CRC for temperature
        if (_crc8(&data[3], 2) != data[5]) return false;
        
        differential_pressure_raw = (data[0] << 8) | data[1];
        temperature_raw = (data[3] << 8) | data[4];
        
        return true;
    }

    float Sensor::_calculateAirFlow(float differential_pressure) {
        // Simplified calculation - adjust based on your duct size
        // Flow = velocity * area, with velocity calculated from pressure
        const float density = 1.225f;  // Air density kg/m³
        const float area = 0.001f;     // Duct cross-sectional area m²
        
        if (differential_pressure >= 0) {
            return sqrt((2 * fabs(differential_pressure)) / density) * area;
        }
        return -sqrt((2 * fabs(differential_pressure)) / density) * area;
    }

    float Sensor::_calculateAirVelocity(float differential_pressure) {
        const float density = 1.225f;  // Air density kg/m³
        
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