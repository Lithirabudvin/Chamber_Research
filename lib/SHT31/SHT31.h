#pragma once

#include <Wire.h>
#include <Adafruit_SHT31.h>

namespace SHT31 {
    
    struct Data {
        float temperature;   // °C
        float humidity;      // %RH
        bool valid;          // Keep this for data validity
    };

    class Sensor {
    public:
        Sensor(TwoWire& wire, uint8_t address = 0x44);
        
        bool begin();
        bool readMeasurement(Data& data);
        bool isInitialized() const { return _initialized; }
        
    private:
        Adafruit_SHT31 _sht31;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
    };
}