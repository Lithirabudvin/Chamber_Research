#include "SGP41.h"

namespace SGP41 {

    Sensor::Sensor(TwoWire& wire, uint8_t address)
        : _wire(wire),
          _address(address),
          _initialized(false),
          _conditioning(true),
          _conditioningTime(10) {
        _sgp41.begin(_wire);
    }

    bool Sensor::begin() {
        uint16_t serial[3];
        int16_t error = _sgp41.getSerialNumber(serial);

        if (error != 0) {
            _initialized = false;
            return false;
        }

        _initialized = true;
        return true;
    }

    bool Sensor::measureRawSignals(Data& data, float temperature, float humidity) {
        if (!_initialized) return false;

        if (_conditioning && _conditioningTime > 0) {
            return executeConditioning(data, temperature, humidity);
        }

        uint16_t rh_ticks = convertHumidity(humidity);
        uint16_t t_ticks  = convertTemperature(temperature);
        uint16_t srawVoc  = 0;
        uint16_t srawNox  = 0;

        int16_t error = _sgp41.measureRawSignals(rh_ticks, t_ticks, srawVoc, srawNox);

        if (error == 0) {
            int32_t vocIndex = _vocAlgorithm.process(srawVoc);
            int32_t noxIndex = _noxAlgorithm.process(srawNox);

            data.voc          = srawVoc;
            data.nox          = srawNox;
            data.vocIndex     = (float)vocIndex;
            data.noxIndex     = (float)noxIndex;
            data.valid        = true;
            data.errorCount   = 0;
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
        uint16_t t_ticks  = convertTemperature(temperature);
        uint16_t srawVoc  = 0;

        int16_t error = _sgp41.executeConditioning(rh_ticks, t_ticks, srawVoc);

        if (error == 0) {
            _conditioningTime--;

            int32_t vocIndex = _vocAlgorithm.process(srawVoc);

            if (_conditioningTime == 0) {
                _conditioning     = false;
                data.conditioning = false;
            } else {
                data.conditioning = true;
            }

            data.voc        = srawVoc;
            data.nox        = 0;
            data.vocIndex   = (float)vocIndex;
            data.noxIndex   = 0.0f;
            data.valid      = true;
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
        return _sgp41.getSerialNumber(serial) == 0;
    }

    bool Sensor::turnHeaterOff() {
        if (!_initialized) return false;
        return _sgp41.turnHeaterOff() == 0;
    }

    bool Sensor::reset() {
        _conditioning     = true;
        _conditioningTime = 10;

        _vocAlgorithm.reset();
        _noxAlgorithm.reset();

        return true;
    }

    uint16_t Sensor::convertTemperature(float temperature) {
        int32_t ticks =
            (int32_t)((temperature + 45.0f) * 65535.0f / 175.0f);

        if (ticks > 65535) ticks = 65535;
        if (ticks < 0)     ticks = 0;

        return (uint16_t)ticks;
    }

    uint16_t Sensor::convertHumidity(float humidity) {
        int32_t ticks =
            (int32_t)(humidity * 65535.0f / 100.0f);

        if (ticks > 65535) ticks = 65535;
        if (ticks < 0)     ticks = 0;

        return (uint16_t)ticks;
    }

} // namespace SGP41