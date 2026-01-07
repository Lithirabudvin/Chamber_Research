#pragma once

#include <Wire.h>
#include <SensirionI2cScd4x.h>

namespace SCD40 {
    
    struct Data {
        uint16_t co2;        // ppm
        float temperature;   // °C
        float humidity;      // %RH
        bool valid;
        int errorCount;
        bool dataReady;
        unsigned long lastRead;
    };

    class Sensor {
    public:
        Sensor(TwoWire& wire, uint8_t address = 0x62);
        
        bool begin();
        bool startPeriodicMeasurement();
        bool stopPeriodicMeasurement();
        bool readMeasurement(Data& data);
        bool getDataReadyStatus(bool& ready);
        bool setAutomaticSelfCalibration(bool enable);
        bool getSerialNumber(uint64_t& serial);
        bool performFactoryReset();
        bool performForcedRecalibration(uint16_t targetCo2Concentration);
        
    private:
        SensirionI2cScd4x _scd40;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
    };
}