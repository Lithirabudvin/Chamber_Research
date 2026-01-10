#include "SFA30.h"

namespace SFA30 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false), _measurementStarted(false) {
        _sfa.begin(_wire, _address);
    }

    bool Sensor::begin() {
        // Try multiple times with error recovery
        for (int attempt = 0; attempt < 3; attempt++) {
            int8_t deviceMarking[32] = {0};
            int16_t error = _sfa.getDeviceMarking(deviceMarking, 32);
            
            if (error == 0) {
                _initialized = true;
                _errorCount = 0;
                return true;
            }
            
            if (attempt < 2) {
                delay(50 + attempt * 50); // Increasing delay
                // Reset the I2C communication
                _wire.endTransmission();
                delay(10);
            }
        }
        
        _initialized = false;
        return false;
    }

    bool Sensor::startContinuousMeasurement() {
        if (!_initialized) {
            if (!begin()) {
                return false;
            }
        }
        
        for (int attempt = 0; attempt < 2; attempt++) {
            int16_t error = _sfa.startContinuousMeasurement();
            if (error == 0) {
                _measurementStarted = true;
                delay(50);  // Allow sensor to start measurements
                return true;
            }
            
            if (attempt < 1) {
                delay(100);
                // Try to reinitialize
                begin();
            }
        }
        
        return false;
    }

    bool Sensor::stopContinuousMeasurement() {
        if (!_initialized) return false;
        
        int16_t error = _sfa.stopMeasurement();
        if (error == 0) {
            _measurementStarted = false;
            return true;
        }
        return false;
    }

    bool Sensor::readMeasurement(Data& data) {
        if (!_initialized) {
            // Try to auto-recover
            if (!begin()) {
                data.valid = false;
                return false;
            }
        }
        
        if (!_measurementStarted) {
            if (!startContinuousMeasurement()) {
                data.valid = false;
                return false;
            }
        }
        
        float hcho = 0, humidity = 0, temperature = 0;
        int16_t error = _sfa.readMeasuredValues(hcho, humidity, temperature);
        
        if (error == 0) {
            // Check for invalid readings (sensor disconnected but still responding)
            if (hcho == 0.0 && humidity == 0.0 && temperature == 0.0) {
                // This could be a disconnected sensor or genuine zeros
                _errorCount++;
                if (_errorCount > 2) {
                    Serial.println("[SFA30] Sensor returning zeros - may be disconnected");
                    data.valid = false;
                    return false;
                }
            } else {
                _errorCount = 0; // Reset on valid reading
            }
            
            data.formaldehyde = hcho;
            data.humidity = humidity;
            data.temperature = temperature;
            data.valid = true;
            return true;
        } else {
            data.valid = false;
            _errorCount++;
            
            // If too many errors, try to reinitialize
            if (_errorCount > 5) {
                Serial.print("[SFA30] Too many errors (");
                Serial.print(_errorCount);
                Serial.println("), attempting reinitialization...");
                _initialized = false;
                _measurementStarted = false;
                _errorCount = 0;
            }
            return false;
        }
    }

    bool Sensor::getDeviceMarking(char* deviceMarking, size_t size) {
        if (!_initialized) return false;
        
        int8_t marking[32];
        int16_t error = _sfa.getDeviceMarking(marking, 32);
        if (error == 0) {
            size_t copySize = min(size - 1, (size_t)31);
            memcpy(deviceMarking, marking, copySize);
            deviceMarking[copySize] = '\0';
            return true;
        }
        return false;
    }

    bool Sensor::reset() {
        // SFA30 doesn't have a reset function, so we'll stop and restart measurements
        stopContinuousMeasurement();
        delay(100);
        return startContinuousMeasurement();
    }
}