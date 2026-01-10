#include "SensorManager.h"

SensorManager::SensorManager(const I2CConfig& i2cConfig, const PMSConfig& pmsConfig)
    : _i2cConfig(i2cConfig), _pmsConfig(pmsConfig),
      _pmsSensor1(Serial2, pmsConfig.rxPin1, pmsConfig.txPin1),
      _pmsSensor2(Serial1, pmsConfig.rxPin2, pmsConfig.txPin2),
      // Main bus sensors (Wire - pins 21/22)
      _sdpSensor(Wire, 0x25),
      _scdSensor(Wire, 0x62),
      _sfaSensor1(Wire, 0x5D),
      _sgpSensor1(Wire, 0x59),
      _shtSensor1(Wire, 0x44),
      _shtSensor2(Wire, 0x45),
      // Alternate bus sensors (Wire1 - pins 25/26)
      _sfaSensor2(_altWire, 0x5D),
      _sgpSensor2(_altWire, 0x59),
      _altWire(1),  // Wire1 instance
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
    
    // First, recover I2C bus in case it's stuck
    _recoverI2CBus();
    
    // Initialize main I2C bus (pins 21/22)
    Serial.printf("  Main I2C: SDA=%d, SCL=%d\n", _i2cConfig.mainSDA, _i2cConfig.mainSCL);
    Wire.begin(_i2cConfig.mainSDA, _i2cConfig.mainSCL);
    Wire.setClock(100000);
    delay(100);
    
    // Initialize alternate I2C bus (pins 25/26)
    Serial.printf("  Alt I2C:  SDA=%d, SCL=%d\n", _i2cConfig.altSDA, _i2cConfig.altSCL);
    _altWire.begin(_i2cConfig.altSDA, _i2cConfig.altSCL);
    _altWire.setClock(100000);
    delay(100);
    
    // Initialize PMS5003 sensors
    Serial.println("\n[PMS5003] Initializing...");
    _pmsActive1 = _pmsSensor1.begin();
    delay(100);
    _pmsActive2 = _pmsSensor2.begin();
    
    // ========== SDP810 INITIALIZATION ==========
    Serial.println("\n[SDP810] Initializing...");
    
    // Try multiple times to initialize SDP810
    bool sdpInitialized = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("  Attempt %d/3: ", attempt);
        
        // Send soft reset
        if (_sdpSensor.softReset()) {
            Serial.print("Reset OK, ");
            delay(5);
        } else {
            Serial.print("Reset failed, ");
        }
        
        // Try to detect sensor
        if (_sdpSensor.begin()) {
            Serial.print("Sensor detected, ");
            
            // Try to start measurement with retry
            bool measurementStarted = false;
            for (int measAttempt = 1; measAttempt <= 2; measAttempt++) {
                if (_sdpSensor.startContinuousMeasurement()) {
                    measurementStarted = true;
                    Serial.print("Measurement started");
                    
                    // Test read to verify
                    SDP810::Data testData;
                    if (_sdpSensor.readMeasurement(testData)) {
                        Serial.printf(" ✓ (P=%+7.2f Pa, T=%6.2f°C)", 
                                    testData.differential_pressure, testData.temperature);
                    } else {
                        Serial.print(" (Test read failed)");
                    }
                    break;
                } else if (measAttempt < 2) {
                    Serial.print("Start failed, retrying... ");
                    delay(50);
                }
            }
            
            if (measurementStarted) {
                sdpInitialized = true;
                _sdpActive = true;
                break;
            } else {
                Serial.println();
            }
        } else {
            Serial.println("Sensor not detected");
        }
        
        delay(100);
    }
    
    if (!sdpInitialized) {
        Serial.println("  ✗ SDP810 initialization failed after 3 attempts");
        _sdpActive = false;
    }
    // ========== END SDP810 INITIALIZATION ==========
    
    // Initialize SCD40
    Serial.println("\n[SCD40] Initializing...");
    _scdActive = _scdSensor.begin();
    if (_scdActive) {
        _scdActive = _scdSensor.startPeriodicMeasurement();
        _scdSensor.setAutomaticSelfCalibration(true);
        Serial.println("  ✓ SCD40 initialized");
    } else {
        Serial.println("  ✗ SCD40 not detected");
    }
    
    // Initialize SHT31 sensors
    Serial.println("\n[SHT31] Initializing...");
    _shtActive1 = _shtSensor1.begin();
    _shtActive2 = _shtSensor2.begin();
    
    if (_shtActive1) {
        _shtSensor1.enableHeater(false);
        Serial.println("  ✓ SHT31 #1 initialized");
    } else {
        Serial.println("  ✗ SHT31 #1 not detected");
    }
    
    if (_shtActive2) {
        _shtSensor2.enableHeater(false);
        Serial.println("  ✓ SHT31 #2 initialized");
    } else {
        Serial.println("  ✗ SHT31 #2 not detected");
    }
    
    // Initialize SFA30 sensors
    Serial.println("\n[SFA30] Initializing...");
    
    // SFA30 #1 on main bus (21/22)
    Serial.println("  SFA30 #1: Main bus (SDA=21, SCL=22)");
    _sfaActive1 = _sfaSensor1.begin();
    if (_sfaActive1) {
        _sfaActive1 = _sfaSensor1.startContinuousMeasurement();
        Serial.println("    ✓ SFA30 #1 initialized");
    } else {
        Serial.println("    ✗ SFA30 #1 not detected");
    }
    
    // SFA30 #2 on alternate bus (25/26)
    Serial.println("  SFA30 #2: Alternate bus (SDA=25, SCL=26)");
    _sfaActive2 = _sfaSensor2.begin();
    if (_sfaActive2) {
        _sfaActive2 = _sfaSensor2.startContinuousMeasurement();
        Serial.println("    ✓ SFA30 #2 initialized");
    } else {
        Serial.println("    ✗ SFA30 #2 not detected");
    }
    
    // Initialize SGP41 sensors
    Serial.println("\n[SGP41] Initializing...");
    
    // SGP41 #1 on main bus (21/22)
    Serial.println("  SGP41 #1: Main bus (SDA=21, SCL=22)");
    _sgpActive1 = _sgpSensor1.begin();
    if (_sgpActive1) {
        Serial.println("    ✓ SGP41 #1 initialized");
    } else {
        Serial.println("    ✗ SGP41 #1 not detected");
    }
    
    // SGP41 #2 on alternate bus (25/26)
    Serial.println("  SGP41 #2: Alternate bus (SDA=25, SCL=26)");
    _sgpActive2 = _sgpSensor2.begin();
    if (_sgpActive2) {
        Serial.println("    ✓ SGP41 #2 initialized");
    } else {
        Serial.println("    ✗ SGP41 #2 not detected");
    }
    
    // Print summary
    Serial.println("\n[SensorManager] Initialization complete!");
    Serial.println("=== SENSOR STATUS ===");
    Serial.printf("  PMS5003 #1: %s\n", _pmsActive1 ? "OK" : "FAIL");
    Serial.printf("  PMS5003 #2: %s\n", _pmsActive2 ? "OK" : "FAIL");
    Serial.printf("  SDP810:     %s\n", _sdpActive ? "OK" : "FAIL");
    Serial.printf("  SCD40:      %s\n", _scdActive ? "OK" : "FAIL");
    Serial.printf("  SFA30 #1:   %s\n", _sfaActive1 ? "OK" : "FAIL");
    Serial.printf("  SFA30 #2:   %s\n", _sfaActive2 ? "OK" : "FAIL");
    Serial.printf("  SGP41 #1:   %s\n", _sgpActive1 ? "OK" : "FAIL");
    Serial.printf("  SGP41 #2:   %s\n", _sgpActive2 ? "OK" : "FAIL");
    Serial.printf("  SHT31 #1:   %s\n", _shtActive1 ? "OK" : "FAIL");
    Serial.printf("  SHT31 #2:   %s\n", _shtActive2 ? "OK" : "FAIL");
    Serial.println("======================");
    
    return true;
}

void SensorManager::_recoverI2CBus() {
    Serial.println("[I2C Recovery] Attempting bus recovery...");
    
    pinMode(_i2cConfig.mainSDA, INPUT_PULLUP);
    pinMode(_i2cConfig.mainSCL, INPUT_PULLUP);
    delay(100);
    
    if (digitalRead(_i2cConfig.mainSDA) == LOW || digitalRead(_i2cConfig.mainSCL) == LOW) {
        Serial.println("  I2C bus appears stuck. Attempting clock recovery...");
        
        pinMode(_i2cConfig.mainSDA, OUTPUT);
        digitalWrite(_i2cConfig.mainSDA, HIGH);
        pinMode(_i2cConfig.mainSCL, OUTPUT);
        
        for (int i = 0; i < 10; i++) {
            digitalWrite(_i2cConfig.mainSCL, LOW);
            delayMicroseconds(5);
            digitalWrite(_i2cConfig.mainSCL, HIGH);
            delayMicroseconds(5);
        }
        
        digitalWrite(_i2cConfig.mainSDA, LOW);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.mainSCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.mainSDA, HIGH);
        delayMicroseconds(5);
        
        pinMode(_i2cConfig.mainSDA, INPUT_PULLUP);
        pinMode(_i2cConfig.mainSCL, INPUT_PULLUP);
        
        Serial.println("  Clock recovery complete");
    } else {
        Serial.println("  I2C bus appears free");
    }
    
    delay(50);
}

bool SensorManager::readSDP() {
    if (!_sdpActive) return false;
    
    bool success = _sdpSensor.readMeasurement(_sdpData);
    
    // If reading fails, try to recover
    if (!success && _sdpActive) {
        Serial.println("[SDP810] Read failed, attempting recovery...");
        
        _sdpSensor.stopContinuousMeasurement();
        delay(50);
        _sdpActive = _sdpSensor.startContinuousMeasurement();
        delay(50);
        
        if (_sdpActive) {
            success = _sdpSensor.readMeasurement(_sdpData);
            if (success) {
                Serial.println("[SDP810] ✓ Recovery successful");
            } else {
                Serial.println("[SDP810] ✗ Recovery failed");
                _sdpActive = false;
            }
        }
    }
    
    return success;
}

bool SensorManager::readPMS1() {
    if (!_pmsActive1) {
        _pmsData1.valid = false;
        return false;
    }
    
    PMS5003::Data newData;
    bool success = _pmsSensor1.readDataNonBlocking(newData);
    
    if (success) {
        _pmsData1 = newData;
        return true;
    } else {
        if (!_pmsSensor1.isDataFresh(PMS_DATA_TIMEOUT)) {
            _pmsData1.valid = false;
            
            if (_pmsSensor1.getTimeSinceLastRead() > PMS_DISCONNECTION_TIMEOUT) {
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
        _pmsData2 = newData;
        return true;
    } else {
        if (!_pmsSensor2.isDataFresh(PMS_DATA_TIMEOUT)) {
            _pmsData2.valid = false;
            
            if (_pmsSensor2.getTimeSinceLastRead() > PMS_DISCONNECTION_TIMEOUT) {
                Serial.println("[PMS5003 #2] ⚠ Sensor not responding - may be disconnected!");
            }
        }
        return false;
    }
}

bool SensorManager::readSCD() {
    if (!_scdActive) return false;
    return _scdSensor.readMeasurement(_scdData);
}

bool SensorManager::readSFA1() {
    if (!_sfaActive1) return false;
    // Uses Wire (main bus) - NO BUS SWITCHING
    return _sfaSensor1.readMeasurement(_sfaData1);
}

bool SensorManager::readSFA2() {
    if (!_sfaActive2) return false;
    // Uses _altWire (alternate bus) - NO BUS SWITCHING
    return _sfaSensor2.readMeasurement(_sfaData2);
}

bool SensorManager::readSGP1() {
    if (!_sgpActive1) return false;
    
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    // Uses Wire (main bus) - NO BUS SWITCHING
    return _sgpSensor1.measureRawSignals(_sgpData1, temp, hum);
}

bool SensorManager::readSGP2() {
    if (!_sgpActive2) return false;
    
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    // Uses _altWire (alternate bus) - NO BUS SWITCHING
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
        _shtData1.heaterEnabled = _shtHeaterEnabled1;
        Serial.printf("[SHT31 #1] Heater %s\n", _shtHeaterEnabled1 ? "ON" : "OFF");
    }
    
    if (_shtActive2 && now - _lastHeaterToggle2 >= HEATER_INTERVAL) {
        _lastHeaterToggle2 = now;
        _shtHeaterEnabled2 = !_shtHeaterEnabled2;
        _shtSensor2.enableHeater(_shtHeaterEnabled2);
        _shtData2.heaterEnabled = _shtHeaterEnabled2;
        Serial.printf("[SHT31 #2] Heater %s\n", _shtHeaterEnabled2 ? "ON" : "OFF");
    }
}

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