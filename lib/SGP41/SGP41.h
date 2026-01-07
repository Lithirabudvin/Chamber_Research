#pragma once

#include <Wire.h>
#include <SensirionI2CSgp41.h>

namespace SGP41 {
    
    struct Data {
        uint16_t voc;           // VOC signal (raw)
        uint16_t nox;           // NOx signal (raw)
        float vocIndex;         // Calculated VOC index (0-500)
        float noxIndex;         // Calculated NOx index (1-5)
        bool valid;
        int errorCount;
        bool conditioning;      // True during first 10 seconds
    };

    class Sensor {
    public:
        Sensor(TwoWire& wire, uint8_t address = 0x59);
        
        bool begin();
        bool measureRawSignals(Data& data, float temperature, float humidity);
        bool executeConditioning(Data& data, float temperature, float humidity);
        bool getSerialNumber(uint16_t* serial, size_t size);
        bool turnHeaterOff();
        bool reset();
        bool isConditioningComplete() const { return !_conditioning; }
        
        static uint16_t convertTemperature(float temperature);
        static uint16_t convertHumidity(float humidity);
        static float calculateVOCIndex(uint16_t rawVoc, float vocBaseline = 100.0);
        static float calculateNOxIndex(uint16_t rawNox, float noxBaseline = 100.0);
        
    private:
        SensirionI2CSgp41 _sgp41;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
        bool _conditioning;
        uint16_t _conditioningTime;
    };
}