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
        bool softReset();      // New: Gentle recovery
        bool hardReset();      // New: Complete reinitialization
        
        bool isInitialized() const { return _initialized; }
        bool isMeasurementStarted() const { return _measurementStarted; }
        int getErrorCount() const { return _errorCount; }
        int getConsecutiveZeros() const { return _consecutiveZeros; }
        unsigned long getTimeSinceLastSuccess() const { 
            return _lastSuccessfulRead > 0 ? millis() - _lastSuccessfulRead : 0; 
        }
        
    private:
        SensirionI2cSfa3x _sfa;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
        bool _measurementStarted;
        int _errorCount;
        int _consecutiveZeros;
        unsigned long _lastSuccessfulRead;
    };
}