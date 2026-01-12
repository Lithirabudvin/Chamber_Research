#include "SHT31.h"

namespace SHT31 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false) {
    }

    bool Sensor::begin() {
        _initialized = _sht31.begin(_address);
        return _initialized;
    }

    bool Sensor::readMeasurement(Data& data) {
        if (!_initialized) return false;
        
        float temperature = _sht31.readTemperature();
        float humidity = _sht31.readHumidity();
        
        if (!isnan(temperature) && !isnan(humidity)) {
            data.temperature = temperature;
            data.humidity = humidity;
            data.valid = true;
            return true;
        } else {
            data.valid = false;
            return false;
        }
    }
}