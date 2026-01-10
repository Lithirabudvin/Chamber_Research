#pragma once

#include <Wire.h>
#include <SensirionI2cSfa3x.h>

namespace SFA30 {
    
    struct Data {
        float formaldehyde;  // ppb
        float humidity;      // %RH
        float temperature;   // °C
        bool valid;
    };

    class Sensor {
    public:
        Sensor(TwoWire& wire, uint8_t address = 0x5D);
        
        bool begin();
        bool startContinuousMeasurement();
        bool stopContinuousMeasurement();
        bool readMeasurement(Data& data);
        bool getDeviceMarking(char* deviceMarking, size_t size);
        bool reset();
        bool isInitialized() const { return _initialized; }
        bool isMeasurementStarted() const { return _measurementStarted; }
        
    private:
        SensirionI2cSfa3x _sfa;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
        bool _measurementStarted;
        int _errorCount = 0;
    };
}