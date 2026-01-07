#include "SGP41.h"

namespace SGP41 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false), 
          _conditioning(true), _conditioningTime(10) {
        _sgp41.begin(_wire);
    }

    bool Sensor::begin() {
        uint16_t serial[3];
        int16_t error = _sgp41.getSerialNumber(serial);
        
        _initialized = (error == 0);
        return _initialized;
    }

    bool Sensor::measureRawSignals(Data& data, float temperature, float humidity) {
        if (!_initialized) return false;
        
        if (_conditioning && _conditioningTime > 0) {
            return executeConditioning(data, temperature, humidity);
        }
        
        uint16_t rh_ticks = convertHumidity(humidity);
        uint16_t t_ticks = convertTemperature(temperature);
        uint16_t srawVoc = 0, srawNox = 0;
        
        int16_t error = _sgp41.measureRawSignals(rh_ticks, t_ticks, srawVoc, srawNox);
        
        if (error == 0) {
            data.voc = srawVoc;
            data.nox = srawNox;
            data.vocIndex = calculateVOCIndex(srawVoc);
            data.noxIndex = calculateNOxIndex(srawNox);
            data.valid = true;
            data.errorCount = 0;
            data.conditioning = false;
            return true;
        } else {
            data.valid = false;
            data.errorCount++;
            return false;
        }
    }

    bool Sensor::executeConditioning(Data& data, float temperature, float humidity) {
        if (!_initialized) return false;
        
        uint16_t rh_ticks = convertHumidity(humidity);
        uint16_t t_ticks = convertTemperature(temperature);
        uint16_t srawVoc = 0;
        
        int16_t error = _sgp41.executeConditioning(rh_ticks, t_ticks, srawVoc);
        
        if (error == 0) {
            _conditioningTime--;
            if (_conditioningTime == 0) {
                _conditioning = false;
                data.conditioning = false;
            } else {
                data.conditioning = true;
            }
            
            data.voc = srawVoc;
            data.nox = 0;
            data.vocIndex = calculateVOCIndex(srawVoc);
            data.noxIndex = 0.0;
            data.valid = true;
            data.errorCount = 0;
            return true;
        } else {
            data.valid = false;
            data.errorCount++;
            return false;
        }
    }

    bool Sensor::getSerialNumber(uint16_t* serial, size_t size) {
        if (!_initialized) return false;
        int16_t error = _sgp41.getSerialNumber(serial);
        return error == 0;
    }

    bool Sensor::turnHeaterOff() {
        if (!_initialized) return false;
        int16_t error = _sgp41.turnHeaterOff();
        return error == 0;
    }

    bool Sensor::reset() {
        // SGP41 doesn't have a reset function, restart conditioning instead
        _conditioning = true;
        _conditioningTime = 10;
        return true;
    }

    uint16_t Sensor::convertTemperature(float temperature) {
        int32_t ticks = (int32_t)((temperature + 45) * 65535.0 / 175.0);
        if (ticks > 65535) ticks = 65535;
        if (ticks < 0) ticks = 0;
        return (uint16_t)ticks;
    }

    uint16_t Sensor::convertHumidity(float humidity) {
        int32_t ticks = (int32_t)(humidity * 65535.0 / 100.0);
        if (ticks > 65535) ticks = 65535;
        if (ticks < 0) ticks = 0;
        return (uint16_t)ticks;
    }

    float Sensor::calculateVOCIndex(uint16_t rawVoc, float vocBaseline) {
        if (rawVoc > 65000) return 500.0;
        return (rawVoc / 650.0);
    }

    float Sensor::calculateNOxIndex(uint16_t rawNox, float noxBaseline) {
        if (rawNox > 65000) return 5.0;
        return 1.0 + (rawNox / 13000.0);
    }
}