#pragma once

#include <Wire.h>

namespace SDP810 {
    
    enum MeasurementMode {
        CONTINUOUS_MEASUREMENT = 0x3603,
        CONTINUOUS_MEASUREMENT_AVG = 0x3615,
        CONTINUOUS_MEASUREMENT_DIFF = 0x362F
    };

    struct Data {
        float differential_pressure;  // Pa
        float temperature;           // °C
        float air_flow;              // m³/s
        float air_velocity;          // m/s
        bool valid;
    };

    class Sensor {
    public:
        Sensor(TwoWire& wire, uint8_t address = 0x25);
        
        bool begin();
        bool startContinuousMeasurement(MeasurementMode mode = CONTINUOUS_MEASUREMENT);
        bool readMeasurement(Data& data);
        bool stopContinuousMeasurement();
        bool readSimple(float& pressure, float& temperature);  // ADD THIS
        uint8_t getAddress() const { return _address; }
        
    private:
        TwoWire& _wire;
        uint8_t _address;
        bool _continuousMode;
        
        bool _readRawValues(int16_t& differential_pressure_raw, int16_t& temperature_raw);
        float _calculateAirFlow(float differential_pressure);
        float _calculateAirVelocity(float differential_pressure);
        uint8_t _crc8(const uint8_t* data, size_t length);
    };
}