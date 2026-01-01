#ifndef PMS5003_H
#define PMS5003_H

#include <Arduino.h>

class PMS5003 {
public:
    struct Data {
        uint16_t pm10_standard;      // PM1.0 concentration (μg/m³)
        uint16_t pm25_standard;      // PM2.5 concentration (μg/m³)
        uint16_t pm100_standard;     // PM10.0 concentration (μg/m³)
        uint16_t pm10_env;           // PM1.0 concentration (atmospheric)
        uint16_t pm25_env;           // PM2.5 concentration (atmospheric)
        uint16_t pm100_env;          // PM10.0 concentration (atmospheric)
        uint16_t particles_03um;     // Particles > 0.3μm / 0.1L
        uint16_t particles_05um;     // Particles > 0.5μm / 0.1L
        uint16_t particles_10um;     // Particles > 1.0μm / 0.1L
        uint16_t particles_25um;     // Particles > 2.5μm / 0.1L
        uint16_t particles_50um;     // Particles > 5.0μm / 0.1L
        uint16_t particles_100um;    // Particles > 10.0μm / 0.1L
    };
    
    PMS5003(HardwareSerial& serial, int rxPin = 16, int txPin = 17);
    bool begin();
    bool readData(Data& data);
    bool isAvailable();
    int calculateAQI(float pm25);
    String getAQICategory(int aqi);
    String getAQIAdvice(int aqi);
    void sleep();
    void wakeUp();
    
private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    bool _parseData(uint8_t* buffer, Data& data);
    bool _checksumValid(uint8_t* buffer);
    float _mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif