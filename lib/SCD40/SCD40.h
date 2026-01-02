#ifndef SCD40_H
#define SCD40_H

#include <Arduino.h>
#include <Wire.h>

class SCD40 {
public:
    struct Data {
        float co2;          // CO₂ concentration (ppm)
        float temperature;  // Temperature (°C)
        float humidity;     // Relative humidity (%)
        uint16_t raw_co2;
        uint16_t raw_temp;
        uint16_t raw_humidity;
        uint8_t status;
    };
    
    enum MeasurementMode {
        SINGLE_SHOT = 0,
        CONTINUOUS = 1,
        LOW_POWER_CONTINUOUS = 2
    };
    
    enum CalibrationMode {
        AUTO_CALIBRATION_DISABLED = 0,
        AUTO_CALIBRATION_ENABLED = 400  // Default 400ppm
    };
    
    SCD40(TwoWire& wire = Wire, uint8_t address = 0x62);
    
    // Basic operations
    bool begin();
    bool softReset();
    bool startPeriodicMeasurement(uint16_t interval_seconds = 5);
    bool startLowPowerPeriodicMeasurement(uint16_t interval_seconds = 30);
    bool stopPeriodicMeasurement();
    bool measureSingleShot();
    bool measureSingleShotRHTOnly();
    bool readMeasurement(Data& data);
    bool isDataReady();
    
    // Calibration
    bool setForcedRecalibration(float co2_reference);
    float getForcedRecalibration();
    bool setAutomaticSelfCalibration(bool enabled);
    bool getAutomaticSelfCalibration();
    bool setTemperatureOffset(float offset);
    float getTemperatureOffset();
    bool setSensorAltitude(uint16_t altitude);
    uint16_t getSensorAltitude();
    bool setAmbientPressure(uint16_t pressure);
    
    // Advanced features
    bool persistSettings();
    bool performFactoryReset();
    bool performSelfTest();
    bool reinit();
    
    // Device information
    uint16_t getSerialNumber(uint16_t* serial, uint8_t max_serial = 3);
    String getSerialNumberString();
    uint16_t getFirmwareVersion();
    bool getFeatures(uint16_t* features);
    
    // Status checking
    bool isConnected();
    bool isMeasuring();
    
    // Utility functions
    static String getCO2QualityLabel(float co2_ppm);
    static String getCO2HealthImpact(float co2_ppm);
    static float calculateDewPoint(float temperature, float humidity);
    static float calculateAbsoluteHumidity(float temperature, float humidity);
    
private:
    TwoWire& _wire;
    uint8_t _address;
    bool _measurementActive;
    MeasurementMode _currentMode;
    
    // I2C communication
    bool _writeCommand(uint16_t command);
    bool _writeCommandWithData(uint16_t command, const uint16_t* data, uint8_t data_words);
    bool _readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms = 1);
    bool _readResponseWithCRC(uint16_t* data, uint8_t data_words);
    
    // CRC calculation (Sensirion CRC8)
    static uint8_t _crc8(uint8_t data[], uint8_t len);
    static bool _checkCRC(uint8_t data[], uint8_t len, uint8_t checksum);
    
    // Data conversion
    static float _convertCO2(uint16_t raw);
    static float _convertTemperature(uint16_t raw);
    static float _convertHumidity(uint16_t raw);
    
    // Command definitions (will be defined in .cpp)
    static const uint16_t CMD_START_PERIODIC_MEAS;
    static const uint16_t CMD_STOP_PERIODIC_MEAS;
    static const uint16_t CMD_READ_MEAS;
    static const uint16_t CMD_DATA_READY;
    static const uint16_t CMD_SINGLE_SHOT_MEAS;
    static const uint16_t CMD_SINGLE_SHOT_RHT_ONLY;
    static const uint16_t CMD_SET_TEMP_OFFSET;
    static const uint16_t CMD_GET_TEMP_OFFSET;
    static const uint16_t CMD_SET_ALTITUDE;
    static const uint16_t CMD_GET_ALTITUDE;
    static const uint16_t CMD_SET_AMBIENT_PRESSURE;
    static const uint16_t CMD_FORCED_RECAL;
    static const uint16_t CMD_GET_FORCED_RECAL;
    static const uint16_t CMD_SET_AUTO_CALIB;
    static const uint16_t CMD_GET_AUTO_CALIB;
    static const uint16_t CMD_PERSIST_SETTINGS;
    static const uint16_t CMD_GET_SERIAL;
    static const uint16_t CMD_GET_FIRMWARE;
    static const uint16_t CMD_PERFORM_SELF_TEST;
    static const uint16_t CMD_REINIT;
    static const uint16_t CMD_SOFT_RESET;
    static const uint16_t CMD_LOW_POWER_PERIODIC_MEAS;
};

#endif