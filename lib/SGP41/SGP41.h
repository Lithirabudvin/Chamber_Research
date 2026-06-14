#pragma once

#include <Wire.h>
#include <SensirionI2CSgp41.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

namespace SGP41 {

    struct Data {
        uint16_t voc;           // VOC raw signal (SRAW_VOC)
        uint16_t nox;           // NOx raw signal (SRAW_NOX)
        float vocIndex;         // VOC Index (1–500, nominal 100)
        float noxIndex;         // NOx Index (1–500, nominal 1)
        bool valid;
        int errorCount;
        bool conditioning;      // True during the 10-second conditioning phase
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

    private:
        SensirionI2CSgp41 _sgp41;
        TwoWire& _wire;
        uint8_t _address;
        bool _initialized;
        bool _conditioning;
        uint16_t _conditioningTime;

        VOCGasIndexAlgorithm _vocAlgorithm;
        NOxGasIndexAlgorithm _noxAlgorithm;
    };
}