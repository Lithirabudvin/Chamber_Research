#include "SHT31.h"

namespace SHT31 {

    Sensor::Sensor(TwoWire& wire, uint8_t address) 
        : _wire(wire), _address(address), _initialized(false) {
        // Store wire reference but Adafruit_SHT31 handles its own Wire instance
    }

    bool Sensor::begin() {
        // Adafruit_SHT31 uses its own Wire instance, we need to pass it
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
            data.heaterEnabled = isHeaterEnabled();
            data.valid = true;
            data.errorCount = 0;
            return true;
        } else {
            data.valid = false;
            data.errorCount++;
            return false;
        }
    }

    bool Sensor::enableHeater(bool enable) {
        if (!_initialized) return false;
        _sht31.heater(enable);
        return true;  // heater() returns void, so we assume success
    }

    bool Sensor::isHeaterEnabled() const {
        if (!_initialized) return false;
        // Need to cast away const for Adafruit library
        return const_cast<Adafruit_SHT31&>(_sht31).isHeaterEnabled();
    }

    bool Sensor::reset() {
        if (!_initialized) return false;
        
        // SHT31 soft reset command
        _wire.beginTransmission(_address);
        _wire.write(0x30);
        _wire.write(0xA2);
        uint8_t error = _wire.endTransmission();
        
        if (error == 0) {
            delay(10);
            // Re-initialize
            return begin();
        }
        return false;
    }
}