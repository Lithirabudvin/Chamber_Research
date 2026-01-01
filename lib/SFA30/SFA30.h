#ifndef SFA30_H
#define SFA30_H

#include <Arduino.h>
#include <Wire.h>

class SFA30 {
public:
    enum MeasurementMode {
        SINGLE_SHOT = 0,
        CONTINUOUS = 1
    };
    
    struct Data {
        float formaldehyde;    // Formaldehyde concentration (ppm)
        float humidity;        // Relative humidity (%)
        float temperature;     // Temperature (°C)
        uint16_t raw_hcho;
        uint16_t raw_rh;
        uint16_t raw_temp;
        uint8_t status;
    };
    
    SFA30(TwoWire& wire = Wire, uint8_t address = 0x5D);
    
    // Basic operations
    bool begin();
    bool softReset();
    bool startContinuousMeasurement(MeasurementMode mode = CONTINUOUS);
    bool stopMeasurement();
    bool readMeasurement(Data& data);
    bool triggerSingleMeasurement(Data& data);
    
    // Advanced features
    bool setMeasurementInterval(uint16_t interval_seconds);
    uint16_t getMeasurementInterval();
    bool setTemperatureOffset(float offset);
    float getTemperatureOffset();
    
    // Calibration and maintenance
    bool startFanCleaning();
    bool isFanCleaningActive();
    
    // Device information
    uint32_t getSerialNumber();
    String getSerialNumberString();
    uint8_t getProductType();
    uint8_t getProductVersion();
    
    // Status checking
    bool isConnected();
    bool isDataReady();
    bool hasError();
    
    // Utility functions
    static float convertFormaldehyde(uint16_t raw);
    static float convertHumidity(uint16_t raw);
    static float convertTemperature(uint16_t raw);
    static String getAirQualityLabel(float hcho_ppm);
    
private:
    TwoWire& _wire;
    uint8_t _address;
    MeasurementMode _mode;
    bool _measurementActive;
    
    // I2C communication
    bool _writeCommand(uint16_t command);
    bool _writeCommandWithData(uint16_t command, const uint8_t* data, uint8_t data_len);
    bool _readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms = 10);
    bool _readMeasurementRaw(uint16_t& hcho, uint16_t& rh, uint16_t& temp);
    
    // CRC calculation (Sensirion uses CRC8)
    static uint8_t _crc8(const uint8_t* data, uint8_t len);
    static bool _checkCrc(const uint8_t* data, uint8_t len, uint8_t checksum);
    
    // Command definitions (DECLARE as static const in header)
    static const uint16_t CMD_START_CONT_MEAS;
    static const uint16_t CMD_STOP_MEAS;
    static const uint16_t CMD_READ_MEAS;
    static const uint16_t CMD_SINGLE_MEAS;
    static const uint16_t CMD_SOFT_RESET;
    static const uint16_t CMD_GET_SERIAL;
    static const uint16_t CMD_GET_FEATURES;
    static const uint16_t CMD_SET_INTERVAL;
    static const uint16_t CMD_GET_INTERVAL;
    static const uint16_t CMD_SET_TEMP_OFFSET;
    static const uint16_t CMD_GET_TEMP_OFFSET;
    static const uint16_t CMD_START_FAN_CLEAN;
    static const uint16_t CMD_GET_FAN_CLEAN;
};

#endif