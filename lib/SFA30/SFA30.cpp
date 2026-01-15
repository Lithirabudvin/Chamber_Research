#include "SFA30.h"

namespace SFA30 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false), 
          _measurementStarted(false), _errorCount(0),
          _lastSuccessfulRead(0), _consecutiveZeros(0) {
        _sfa.begin(_wire, _address);
    }

    bool Sensor::begin() {
        // Stop any existing measurements first
        _sfa.stopMeasurement();
        delay(100);
        
        // Try multiple times with increasing delays
        for (int attempt = 0; attempt < 3; attempt++) {
            int8_t deviceMarking[32] = {0};
            int16_t error = _sfa.getDeviceMarking(deviceMarking, 32);
            
            if (error == 0) {
                _initialized = true;
                _errorCount = 0;
                _consecutiveZeros = 0;
                Serial.printf("[SFA30 0x%02X] Initialized successfully\n", _address);
                return true;
            }
            
            if (attempt < 2) {
                delay(100 * (attempt + 1)); // 100ms, 200ms
                // Soft reset the I2C communication
                _wire.endTransmission();
                delay(50);
            }
        }
        
        _initialized = false;
        Serial.printf("[SFA30 0x%02X] Initialization failed\n", _address);
        return false;
    }

    bool Sensor::startContinuousMeasurement() {
        if (!_initialized) {
            if (!begin()) {
                return false;
            }
        }
        
        // Stop any existing measurement
        _sfa.stopMeasurement();
        delay(100);
        
        for (int attempt = 0; attempt < 3; attempt++) {
            int16_t error = _sfa.startContinuousMeasurement();
            if (error == 0) {
                _measurementStarted = true;
                delay(2000);  // CRITICAL: Wait for first measurement (SFA30 needs 2s)
                _lastSuccessfulRead = millis();
                Serial.printf("[SFA30 0x%02X] Measurement started\n", _address);
                return true;
            }
            
            if (attempt < 2) {
                delay(200 * (attempt + 1));
                begin(); // Try to reinitialize
            }
        }
        
        Serial.printf("[SFA30 0x%02X] Failed to start measurement\n", _address);
        return false;
    }

    bool Sensor::stopContinuousMeasurement() {
        if (!_initialized) return false;
        
        int16_t error = _sfa.stopMeasurement();
        if (error == 0) {
            _measurementStarted = false;
            delay(50);
            return true;
        }
        return false;
    }

    bool Sensor::readMeasurement(Data& data) {
        // Check if too much time has passed since last successful read
        unsigned long timeSinceSuccess = millis() - _lastSuccessfulRead;
        if (timeSinceSuccess > 60000 && _lastSuccessfulRead > 0) {
            Serial.printf("[SFA30 0x%02X] No valid data for %lu seconds, forcing reset\n", 
                         _address, timeSinceSuccess / 1000);
            hardReset();
            data.valid = false;
            return false;
        }
        
        if (!_initialized) {
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
            // Check for all zeros (disconnected sensor or communication error)
            if (hcho == 0.0 && humidity == 0.0 && temperature == 0.0) {
                _consecutiveZeros++;
                
                // Only log periodically to avoid spam
                if (_consecutiveZeros == 1 || _consecutiveZeros % 10 == 0) {
                    Serial.printf("[SFA30 0x%02X] Returning zeros (count: %d)\n", 
                                 _address, _consecutiveZeros);
                }
                
                // After 5 consecutive zeros, attempt recovery
                if (_consecutiveZeros >= 5) {
                    Serial.printf("[SFA30 0x%02X] Too many zero readings, attempting recovery\n", 
                                 _address);
                    softReset();
                    data.valid = false;
                    return false;
                }
                
                // Still return as invalid but don't increment error count yet
                data.valid = false;
                return false;
            }
            
            // Valid non-zero reading
            _consecutiveZeros = 0;
            _errorCount = 0;
            _lastSuccessfulRead = millis();
            
            data.formaldehyde = hcho;
            data.humidity = humidity;
            data.temperature = temperature;
            data.valid = true;
            return true;
            
        } else {
            // I2C communication error
            data.valid = false;
            _errorCount++;
            
            // Log error periodically
            if (_errorCount == 1 || _errorCount % 5 == 0) {
                Serial.printf("[SFA30 0x%02X] Read error #%d (code: %d)\n", 
                             _address, _errorCount, error);
            }
            
            // Progressive recovery strategy
            if (_errorCount >= 3 && _errorCount < 10) {
                // Soft reset first
                Serial.printf("[SFA30 0x%02X] Multiple failures, attempting soft reset\n", 
                             _address);
                softReset();
            } else if (_errorCount >= 10) {
                // Hard reset if soft reset doesn't work
                Serial.printf("[SFA30 0x%02X] Critical failure, attempting hard reset\n", 
                             _address);
                hardReset();
            }
            
            return false;
        }
    }

    bool Sensor::softReset() {
        Serial.printf("[SFA30 0x%02X] Performing soft reset...\n", _address);
        
        // Stop measurements
        stopContinuousMeasurement();
        delay(100);
        
        // Clear I2C buffer
        _wire.endTransmission();
        delay(50);
        
        // Restart measurements
        if (startContinuousMeasurement()) {
            _errorCount = 0;
            _consecutiveZeros = 0;
            Serial.printf("[SFA30 0x%02X] Soft reset successful\n", _address);
            return true;
        }
        
        Serial.printf("[SFA30 0x%02X] Soft reset failed\n", _address);
        return false;
    }

    bool Sensor::hardReset() {
        Serial.printf("[SFA30 0x%02X] Performing hard reset...\n", _address);
        
        // Complete reinitialization
        _initialized = false;
        _measurementStarted = false;
        _errorCount = 0;
        _consecutiveZeros = 0;
        _lastSuccessfulRead = 0;
        
        delay(500); // Allow sensor to settle
        
        if (begin() && startContinuousMeasurement()) {
            Serial.printf("[SFA30 0x%02X] Hard reset successful\n", _address);
            return true;
        }
        
        Serial.printf("[SFA30 0x%02X] Hard reset failed\n", _address);
        return false;
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
        return hardReset();
    }
}