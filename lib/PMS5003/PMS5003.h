#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

namespace PMS5003 {
    
    struct Data {
        uint16_t pm10_standard;
        uint16_t pm25_standard;
        uint16_t pm100_standard;
        uint16_t pm10_env;
        uint16_t pm25_env;
        uint16_t pm100_env;
        uint16_t particles_03um;
        uint16_t particles_05um;
        uint16_t particles_10um;
        uint16_t particles_25um;
        uint16_t particles_50um;
        uint16_t particles_100um;
        bool valid;
    };

    class Sensor {
    public:
        Sensor(HardwareSerial& serial, int rxPin, int txPin);
        ~Sensor();
        
        bool begin();
        bool readData(Data& data, uint32_t timeout = 1000);
        bool readDataNonBlocking(Data& data);
        void wakeUp();
        void sleep();
        bool isActive() const { return _active; }
        void setActive(bool active) { _active = active; }
        
    private:
        HardwareSerial& _serial;
        int _rxPin;
        int _txPin;
        bool _active;
        unsigned long _lastRead;
        uint8_t _buffer[32];
        int _bufferIndex;
        
        bool _parseData(Data& data, const uint8_t* buffer);
        uint16_t _calculateChecksum(const uint8_t* data, size_t length);
    };
}