#include "SensorManager.h"

SensorManager::SensorManager(const I2CConfig& i2cConfig, const PMSConfig& pmsConfig)
    : _i2cConfig(i2cConfig), _pmsConfig(pmsConfig),
      _pmsSensor1(Serial2, pmsConfig.rxPin1, pmsConfig.txPin1),
      _pmsSensor2(Serial1, pmsConfig.rxPin2, pmsConfig.txPin2),
      _sdpSensor(Wire, 0x25),
      _scdSensor(Wire, 0x62),
      _sfaSensor1(Wire, 0x5D),
      _sfaSensor2(Wire, 0x5D),
      _sgpSensor1(Wire, 0x59),
      _sgpSensor2(Wire, 0x59),
      _shtSensor1(Wire, 0x44),
      _shtSensor2(Wire, 0x45),
      _pmsActive1(false), _pmsActive2(false),
      _sdpActive(false), _scdActive(false),
      _sfaActive1(false), _sfaActive2(false),
      _sgpActive1(false), _sgpActive2(false),
      _shtActive1(false), _shtActive2(false),
      _lastHeaterToggle1(0), _lastHeaterToggle2(0),
      _shtHeaterEnabled1(false), _shtHeaterEnabled2(false) {
}

bool SensorManager::begin() {
    Serial.println("\n[SensorManager] Initializing...");
    
    // Initialize main I2C bus
    Serial.printf("  Main I2C: SDA=%d, SCL=%d\n", _i2cConfig.mainSDA, _i2cConfig.mainSCL);
    Wire.begin(_i2cConfig.mainSDA, _i2cConfig.mainSCL);
    Wire.setClock(100000);
    delay(100);
    
    // Initialize PMS5003 sensors
    Serial.println("\n[PMS5003] Initializing...");
    _pmsActive1 = _pmsSensor1.begin();
    delay(100);
    _pmsActive2 = _pmsSensor2.begin();
    
    // Initialize SDP810
    Serial.println("\n[SDP810] Initializing...");
    _sdpActive = _sdpSensor.begin();
    if (_sdpActive) {
        _sdpActive = _sdpSensor.startContinuousMeasurement();
    }
    
    // Initialize SCD40
    Serial.println("\n[SCD40] Initializing...");
    _scdActive = _scdSensor.begin();
    if (_scdActive) {
        _scdActive = _scdSensor.startPeriodicMeasurement();
        _scdSensor.setAutomaticSelfCalibration(true);
    }
    
    // Initialize SHT31 sensors
    Serial.println("\n[SHT31] Initializing...");
    _shtActive1 = _shtSensor1.begin();
    _shtActive2 = _shtSensor2.begin();
    
    if (_shtActive1) _shtSensor1.enableHeater(false);
    if (_shtActive2) _shtSensor2.enableHeater(false);
    
    // Initialize SFA30 sensors
    Serial.println("\n[SFA30 #1] Initializing on main bus...");
    _sfaActive1 = _sfaSensor1.begin();
    if (_sfaActive1) {
        _sfaActive1 = _sfaSensor1.startContinuousMeasurement();
    }
    
    // Initialize SFA30 #2 on alternate pins
    Serial.println("\n[SFA30 #2] Initializing on alternate pins...");
    _switchToAltBus();
    _sfaActive2 = _sfaSensor2.begin();
    if (_sfaActive2) {
        _sfaActive2 = _sfaSensor2.startContinuousMeasurement();
    }
    _switchToMainBus();
    
    // Initialize SGP41 sensors
    Serial.println("\n[SGP41] Initializing...");
    _sgpActive1 = _sgpSensor1.begin();
    _sgpActive2 = _sgpSensor2.begin();
    
    Serial.println("\n[SensorManager] Initialization complete!");
    return true;
}

void SensorManager::_switchToMainBus() {
    Wire.end();
    delay(5);
    Wire.begin(_i2cConfig.mainSDA, _i2cConfig.mainSCL);
    delay(5);
}

void SensorManager::_switchToAltBus() {
    Wire.end();
    delay(5);
    Wire.begin(_i2cConfig.altSDA, _i2cConfig.altSCL);
    delay(5);
}

bool SensorManager::readPMS1() {
    if (!_pmsActive1) {
        _pmsData1.valid = false;
        return false;
    }
    
    PMS5003::Data newData;
    bool success = _pmsSensor1.readDataNonBlocking(newData);
    
    if (success) {
        // New data received successfully
        _pmsData1 = newData;
        return true;
    } else {
        // No new data received
        // Check if existing data is still fresh
        if (!_pmsSensor1.isDataFresh(PMS_DATA_TIMEOUT)) {
            // Data is stale - invalidate it
            _pmsData1.valid = false;
            
            // Optional: Check if sensor has stopped responding completely
            if (_pmsSensor1.getTimeSinceLastRead() > PMS_DISCONNECTION_TIMEOUT) {
                // Sensor appears to be disconnected
                Serial.println("[PMS5003 #1] ⚠ Sensor not responding - may be disconnected!");
            }
        }
        return false;
    }
}

bool SensorManager::readPMS2() {
    if (!_pmsActive2) {
        _pmsData2.valid = false;
        return false;
    }
    
    PMS5003::Data newData;
    bool success = _pmsSensor2.readDataNonBlocking(newData);
    
    if (success) {
        // New data received successfully
        _pmsData2 = newData;
        return true;
    } else {
        // No new data received
        // Check if existing data is still fresh
        if (!_pmsSensor2.isDataFresh(PMS_DATA_TIMEOUT)) {
            // Data is stale - invalidate it
            _pmsData2.valid = false;
            
            // Optional: Check if sensor has stopped responding completely
            if (_pmsSensor2.getTimeSinceLastRead() > PMS_DISCONNECTION_TIMEOUT) {
                // Sensor appears to be disconnected
                Serial.println("[PMS5003 #2] ⚠ Sensor not responding - may be disconnected!");
            }
        }
        return false;
    }
}

bool SensorManager::readSDP() {
    if (!_sdpActive) return false;
    return _sdpSensor.readMeasurement(_sdpData);
}

bool SensorManager::readSCD() {
    if (!_scdActive) return false;
    return _scdSensor.readMeasurement(_scdData);
}

bool SensorManager::readSFA1() {
    if (!_sfaActive1) return false;
    return _sfaSensor1.readMeasurement(_sfaData1);
}

bool SensorManager::readSFA2() {
    if (!_sfaActive2) return false;
    
    // Switch to alternate bus for SFA30 #2
    _switchToAltBus();
    bool result = _sfaSensor2.readMeasurement(_sfaData2);
    _switchToMainBus();
    
    return result;
}

bool SensorManager::readSGP1() {
    if (!_sgpActive1) return false;
    
    // Get compensation values from SHT31 sensors
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    return _sgpSensor1.measureRawSignals(_sgpData1, temp, hum);
}

bool SensorManager::readSGP2() {
    if (!_sgpActive2) return false;
    
    // Get compensation values from SHT31 sensors
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    return _sgpSensor2.measureRawSignals(_sgpData2, temp, hum);
}

bool SensorManager::readSHT1() {
    if (!_shtActive1) return false;
    return _shtSensor1.readMeasurement(_shtData1);
}

bool SensorManager::readSHT2() {
    if (!_shtActive2) return false;
    return _shtSensor2.readMeasurement(_shtData2);
}

void SensorManager::manageHeaters() {
    unsigned long now = millis();
    
    if (_shtActive1 && now - _lastHeaterToggle1 >= HEATER_INTERVAL) {
        _lastHeaterToggle1 = now;
        _shtHeaterEnabled1 = !_shtHeaterEnabled1;
        _shtSensor1.enableHeater(_shtHeaterEnabled1);
        // Force read heater status immediately
        _shtData1.heaterEnabled = _shtHeaterEnabled1;
        Serial.printf("[SHT31 #1] Heater %s\n", _shtHeaterEnabled1 ? "ON" : "OFF");
    }
    
    if (_shtActive2 && now - _lastHeaterToggle2 >= HEATER_INTERVAL) {
        _lastHeaterToggle2 = now;
        _shtHeaterEnabled2 = !_shtHeaterEnabled2;
        _shtSensor2.enableHeater(_shtHeaterEnabled2);
        // Force read heater status immediately
        _shtData2.heaterEnabled = _shtHeaterEnabled2;
        Serial.printf("[SHT31 #2] Heater %s\n", _shtHeaterEnabled2 ? "ON" : "OFF");
    }
}

// NEW: PMS sensor health checking methods
bool SensorManager::isPMS1Responding() const {
    return _pmsActive1 && _pmsSensor1.isSensorResponding();
}

bool SensorManager::isPMS2Responding() const {
    return _pmsActive2 && _pmsSensor2.isSensorResponding();
}

unsigned long SensorManager::getPMS1TimeSinceLastRead() const {
    if (!_pmsActive1) return ULONG_MAX;
    return _pmsSensor1.getTimeSinceLastRead();
}

unsigned long SensorManager::getPMS2TimeSinceLastRead() const {
    if (!_pmsActive2) return ULONG_MAX;
    return _pmsSensor2.getTimeSinceLastRead();
}

float SensorManager::getAverageTemperature() const {
    float sum = 0;
    int count = 0;
    
    if (_shtData1.valid) {
        sum += _shtData1.temperature;
        count++;
    }
    if (_shtData2.valid) {
        sum += _shtData2.temperature;
        count++;
    }
    
    return (count > 0) ? sum / count : 25.0;
}

float SensorManager::getAverageHumidity() const {
    float sum = 0;
    int count = 0;
    
    if (_shtData1.valid) {
        sum += _shtData1.humidity;
        count++;
    }
    if (_shtData2.valid) {
        sum += _shtData2.humidity;
        count++;
    }
    
    return (count > 0) ? sum / count : 50.0;
}