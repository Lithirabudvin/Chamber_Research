#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "PMS5003.h"
#include "SDP810.h"
#include "SCD40.h"
#include "SFA30.h"
#include "SGP41.h"
#include "SHT31.h"

class SensorManager {
public:
    // I2C Pin Configuration
    struct I2CConfig {
        int mainSDA = 21;
        int mainSCL = 22;
        int altSDA = 25;
        int altSCL = 26;
    };
    
    // PMS5003 Configuration
    struct PMSConfig {
        int rxPin1 = 16;
        int txPin1 = 17;
        int rxPin2 = 18;
        int txPin2 = 19;
    };
    
    // Initialize with configurations
    SensorManager(const I2CConfig& i2cConfig, const PMSConfig& pmsConfig);
    
    // Main initialization
    bool begin();
    
    // Sensor reading functions
    bool readPMS1();
    bool readPMS2();
    bool readSDP();
    bool readSCD();
    bool readSFA1();
    bool readSFA2();
    bool readSGP1();
    bool readSGP2();
    bool readSHT1();
    bool readSHT2();
    
    // Heater management for SHT31
    void manageHeaters();
    
    // Data access
    const PMS5003::Data& getPMS1Data() const { return _pmsData1; }
    const PMS5003::Data& getPMS2Data() const { return _pmsData2; }
    const SDP810::Data& getSDPData() const { return _sdpData; }
    const SCD40::Data& getSCDData() const { return _scdData; }
    const SFA30::Data& getSFA1Data() const { return _sfaData1; }
    const SFA30::Data& getSFA2Data() const { return _sfaData2; }
    const SGP41::Data& getSGP1Data() const { return _sgpData1; }
    const SGP41::Data& getSGP2Data() const { return _sgpData2; }
    const SHT31::Data& getSHT1Data() const { return _shtData1; }
    const SHT31::Data& getSHT2Data() const { return _shtData2; }
    
    // Sensor status
    bool isPMS1Active() const { return _pmsActive1; }
    bool isPMS2Active() const { return _pmsActive2; }
    bool isSDPActive() const { return _sdpActive; }
    bool isSCDActive() const { return _scdActive; }
    bool isSFA1Active() const { return _sfaActive1; }
    bool isSFA2Active() const { return _sfaActive2; }
    bool isSGP1Active() const { return _sgpActive1; }
    bool isSGP2Active() const { return _sgpActive2; }
    bool isSHT1Active() const { return _shtActive1; }
    bool isSHT2Active() const { return _shtActive2; }
    
    // Utility functions for SGP41 compensation
    float getAverageTemperature() const;
    float getAverageHumidity() const;
    
private:
    // I2C bus management
    void _switchToMainBus();
    void _switchToAltBus();
    
    // Configurations
    I2CConfig _i2cConfig;
    PMSConfig _pmsConfig;
    
    // Sensor objects
    PMS5003::Sensor _pmsSensor1;
    PMS5003::Sensor _pmsSensor2;
    SDP810::Sensor _sdpSensor;
    SCD40::Sensor _scdSensor;
    SFA30::Sensor _sfaSensor1;
    SFA30::Sensor _sfaSensor2;
    SGP41::Sensor _sgpSensor1;
    SGP41::Sensor _sgpSensor2;
    SHT31::Sensor _shtSensor1;
    SHT31::Sensor _shtSensor2;
    
    // Data structures
    PMS5003::Data _pmsData1;
    PMS5003::Data _pmsData2;
    SDP810::Data _sdpData;
    SCD40::Data _scdData;
    SFA30::Data _sfaData1;
    SFA30::Data _sfaData2;
    SGP41::Data _sgpData1;
    SGP41::Data _sgpData2;
    SHT31::Data _shtData1;
    SHT31::Data _shtData2;
    
    // Status flags
    bool _pmsActive1;
    bool _pmsActive2;
    bool _sdpActive;
    bool _scdActive;
    bool _sfaActive1;
    bool _sfaActive2;
    bool _sgpActive1;
    bool _sgpActive2;
    bool _shtActive1;
    bool _shtActive2;
    
    // Heater management
    unsigned long _lastHeaterToggle1;
    unsigned long _lastHeaterToggle2;
    bool _shtHeaterEnabled1;
    bool _shtHeaterEnabled2;
    const unsigned long HEATER_INTERVAL = 30000;
    
    // Error tracking
    const int MAX_ERRORS = 10;
};