#ifndef SCD40_H
#define SCD40_H

#include <Arduino.h>
#include <Wire.h>

class SCD40 {
public:
    struct Data {
        uint16_t co2;          // CO₂ concentration (ppm)
        float temperature;     // Temperature (°C)
        float humidity;        // Relative humidity (%)
        bool valid;            // Whether data is valid
        bool dataReady;        // Whether data is ready to read
        unsigned long timestamp; // When data was read
    };
    
    SCD40(TwoWire& wire = Wire, uint8_t address = 0x62);
    
    // Basic operations
    bool begin();
    bool startMeasurement();
    bool stopMeasurement();
    bool readData(Data& data);
    bool isDataReady();
    
    // Configuration
    bool setTemperatureOffset(float offset);
    float getTemperatureOffset();
    bool setAutomaticSelfCalibration(bool enabled);
    bool getAutomaticSelfCalibration();
    bool setSensorAltitude(uint16_t altitude);
    uint16_t getSensorAltitude();
    bool setAmbientPressure(uint16_t pressure);
    
    // Calibration
    bool performForcedRecalibration(uint16_t target_co2);
    bool performFactoryReset();
    
    // Device information
    uint64_t getSerialNumber();
    String getSerialNumberString();
    uint16_t getFirmwareVersion();
    
    // Status and diagnostics
    bool isConnected();
    bool isMeasuring();
    int getErrorCount() const { return errorCount; }
    void resetErrorCount() { errorCount = 0; }
    
    // Utility functions
    static String getCO2Quality(uint16_t co2);
    static String getCO2Recommendation(uint16_t co2);
    static float calculateDewPoint(float temperature, float humidity);
    static float calculateAbsoluteHumidity(float temperature, float humidity);
    
private:
    TwoWire& wire;
    uint8_t address;
    bool measurementActive;
    int errorCount;
    unsigned long lastReadTime;
    
    // I2C communication
    bool writeCommand(uint16_t command);
    bool writeCommandWithData(uint16_t command, const uint16_t* data, uint8_t data_words);
    bool readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms = 1);
    bool readResponseWithCRC(uint16_t* data, uint8_t data_words);
    
    // CRC calculation
    static uint8_t crc8(uint8_t data[], uint8_t len);
    static bool checkCRC(uint8_t data[], uint8_t len, uint8_t checksum);
    
    // Data conversion
    static float convertTemperature(uint16_t raw);
    static float convertHumidity(uint16_t raw);
    
    // Command definitions
    static const uint16_t CMD_START_PERIODIC_MEAS;
    static const uint16_t CMD_STOP_PERIODIC_MEAS;
    static const uint16_t CMD_READ_MEAS;
    static const uint16_t CMD_DATA_READY;
    static const uint16_t CMD_SET_TEMP_OFFSET;
    static const uint16_t CMD_GET_TEMP_OFFSET;
    static const uint16_t CMD_SET_ALTITUDE;
    static const uint16_t CMD_GET_ALTITUDE;
    static const uint16_t CMD_SET_AMBIENT_PRESSURE;
    static const uint16_t CMD_FORCED_RECAL;
    static const uint16_t CMD_SET_AUTO_CALIB;
    static const uint16_t CMD_GET_AUTO_CALIB;
    static const uint16_t CMD_GET_SERIAL;
    static const uint16_t CMD_GET_FIRMWARE;
    static const uint16_t CMD_PERFORM_FACTORY_RESET;
    static const uint16_t CMD_REINIT;
    static const uint16_t CMD_SOFT_RESET;
};

#endif