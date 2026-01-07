#include "SCD40.h"

namespace SCD40 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false) {
        _scd40.begin(_wire, _address);
    }

    bool Sensor::begin() {
        // Stop any existing measurement
        int16_t error = _scd40.stopPeriodicMeasurement();
        if (error != 0) {
            // Try to recover
            delay(500);
            error = _scd40.stopPeriodicMeasurement();
        }
        
        // Get serial number to verify communication
        uint64_t serial = 0;
        error = _scd40.getSerialNumber(serial);
        
        _initialized = (error == 0);
        return _initialized;
    }

    bool Sensor::startPeriodicMeasurement() {
        if (!_initialized) return false;
        int16_t error = _scd40.startPeriodicMeasurement();
        return error == 0;
    }

    bool Sensor::stopPeriodicMeasurement() {
        if (!_initialized) return false;
        int16_t error = _scd40.stopPeriodicMeasurement();
        return error == 0;
    }

    bool Sensor::readMeasurement(Data& data) {
        if (!_initialized) return false;
        
        bool dataReady = false;
        int16_t error = _scd40.getDataReadyStatus(dataReady);
        if (error != 0) {
            data.valid = false;
            data.errorCount++;
            return false;
        }
        
        if (!dataReady) {
            data.dataReady = false;
            return false;
        }
        
        uint16_t co2 = 0;
        float temperature = 0, humidity = 0;
        
        error = _scd40.readMeasurement(co2, temperature, humidity);
        if (error == 0) {
            data.co2 = co2;
            data.temperature = temperature;
            data.humidity = humidity;
            data.valid = true;
            data.errorCount = 0;
            data.lastRead = millis();
            data.dataReady = true;
            return true;
        } else {
            data.valid = false;
            data.errorCount++;
            return false;
        }
    }

    bool Sensor::getDataReadyStatus(bool& ready) {
        if (!_initialized) return false;
        int16_t error = _scd40.getDataReadyStatus(ready);
        return error == 0;
    }

    bool Sensor::setAutomaticSelfCalibration(bool enable) {
        if (!_initialized) return false;
        int16_t error = _scd40.setAutomaticSelfCalibrationEnabled(enable);
        return error == 0;
    }

    bool Sensor::getSerialNumber(uint64_t& serial) {
        if (!_initialized) return false;
        int16_t error = _scd40.getSerialNumber(serial);
        return error == 0;
    }

    bool Sensor::performFactoryReset() {
        if (!_initialized) return false;
        int16_t error = _scd40.performFactoryReset();
        return error == 0;
    }

    bool Sensor::performForcedRecalibration(uint16_t targetCo2Concentration) {
        if (!_initialized) return false;
        uint16_t frcCorrection;
        int16_t error = _scd40.performForcedRecalibration(targetCo2Concentration, frcCorrection);
        return error == 0;
    }
}