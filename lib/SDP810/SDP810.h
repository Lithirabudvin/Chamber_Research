#ifndef SDP810_H
#define SDP810_H

#include <Arduino.h>
#include <Wire.h>

class SDP810 {
public:
    enum MeasurementMode {
        CONTINUOUS_MEASUREMENT = 0x3615,
        CONTINUOUS_MEASUREMENT_AVG = 0x3721,
        TRIGGERED_MEASUREMENT = 0x3621,
        TRIGGERED_MEASUREMENT_AVG = 0x3721
    };
    
    struct Data {
        float differential_pressure;  // Pascal (Pa)
        float temperature;           // Celsius (°C)
        float air_flow;              // m³/s (calculated)
        float air_velocity;          // m/s (calculated)
        uint16_t raw_pressure;
        uint16_t raw_temperature;
        uint8_t status;
    };
    
    SDP810(TwoWire& wire = Wire, uint8_t address = 0x25);
    bool begin();
    bool startContinuousMeasurement(MeasurementMode mode = CONTINUOUS_MEASUREMENT);
    bool readMeasurement(Data& data);
    bool triggerMeasurement(Data& data);
    bool softReset();
    float calculateAirFlow(float pressure, float diameter = 0.1); // diameter in meters
    float calculateAirVelocity(float pressure, float diameter = 0.1);
    bool isConnected();
    
private:
    TwoWire& _wire;
    uint8_t _address;
    bool _readRawData(uint16_t& pressure, uint16_t& temperature, uint8_t& status);
    float _convertPressure(uint16_t raw);
    float _convertTemperature(uint16_t raw);
    void _writeCommand(uint16_t command);
    bool _readResponse(uint8_t* buffer, uint8_t length);
};

#endif