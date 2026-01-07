#include "SFA30.h"

namespace SFA30 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false), _measurementStarted(false) {
        _sfa.begin(_wire, _address);
    }

    bool Sensor::begin() {
        int8_t deviceMarking[32] = {0};
        int16_t error = _sfa.getDeviceMarking(deviceMarking, 32);
        
        _initialized = (error == 0);
        return _initialized;
    }

    bool Sensor::startContinuousMeasurement() {
        if (!_initialized) return false;
        
        int16_t error = _sfa.startContinuousMeasurement();
        if (error == 0) {
            _measurementStarted = true;
            delay(50);  // Allow sensor to start measurements
        }
        return error == 0;
    }

    bool Sensor::stopContinuousMeasurement() {
        if (!_initialized) return false;
        
        int16_t error = _sfa.stopMeasurement();
        if (error == 0) {
            _measurementStarted = false;
        }
        return error == 0;
    }

    bool Sensor::readMeasurement(Data& data) {
        if (!_initialized || !_measurementStarted) return false;
        
        float hcho = 0, humidity = 0, temperature = 0;
        int16_t error = _sfa.readMeasuredValues(hcho, humidity, temperature);
        
        if (error == 0) {
            data.formaldehyde = hcho;
            data.humidity = humidity;
            data.temperature = temperature;
            data.valid = true;
            data.errorCount = 0;
            return true;
        } else {
            data.valid = false;
            data.errorCount++;
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
        if (!_initialized) return false;
        
        // SFA30 doesn't have a reset function, so we'll stop and restart measurements
        stopContinuousMeasurement();
        delay(100);
        return startContinuousMeasurement();
    }
}