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
      _shtHeaterEnabled1(false), _shtHeaterEnabled2(false),
      _lastMainBusRecovery(0), _lastAltBusRecovery(0) {
}

bool SensorManager::begin() {
    Serial.println("\n[SensorManager] Initializing...");
    
    // Initialize main I2C bus (pins 21/22)
    Serial.printf("  Main I2C: SDA=%d, SCL=%d\n", _i2cConfig.mainSDA, _i2cConfig.mainSCL);
    recoverMainI2CBus();
    Wire.begin(_i2cConfig.mainSDA, _i2cConfig.mainSCL);
    Wire.setClock(100000);
    delay(100);
    
    // Initialize alternate I2C bus (pins 25/26)
    Serial.printf("  Alt I2C:  SDA=%d, SCL=%d\n", _i2cConfig.altSDA, _i2cConfig.altSCL);
    recoverAltI2CBus();
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
    for (int attempt = 1; attempt <= 2; attempt++) {
        _scdActive = _scdSensor.begin();
        if (_scdActive) {
            _scdActive = _scdSensor.startPeriodicMeasurement();
            if (_scdActive) {
                _scdSensor.setAutomaticSelfCalibration(true);
                Serial.println("  ✓ SCD40 initialized");
                break;
            }
        }
        if (attempt < 2) {
            Serial.print("  Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
        } else {
            Serial.println("  ✗ SCD40 not detected");
        }
    }
    
    // Initialize SHT31 sensors
    Serial.println("\n[SHT31] Initializing...");
    for (int attempt = 1; attempt <= 2; attempt++) {
        _shtActive1 = _shtSensor1.begin();
        _shtActive2 = _shtSensor2.begin();
        
        if (_shtActive1 && _shtActive2) {
            _shtSensor1.enableHeater(false);
            _shtSensor2.enableHeater(false);
            Serial.println("  ✓ SHT31 #1 and #2 initialized");
            break;
        }
        if (attempt < 2) {
            Serial.print("  Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
        } else {
            if (!_shtActive1) Serial.println("  ✗ SHT31 #1 not detected");
            if (!_shtActive2) Serial.println("  ✗ SHT31 #2 not detected");
        }
    }
    
    // Initialize SFA30 sensors
    Serial.println("\n[SFA30] Initializing...");
    
    // SFA30 #1 on main bus (21/22)
    Serial.println("  SFA30 #1: Main bus (SDA=21, SCL=22)");
    for (int attempt = 1; attempt <= 3; attempt++) {
        _sfaActive1 = _sfaSensor1.begin();
        if (_sfaActive1) {
            _sfaActive1 = _sfaSensor1.startContinuousMeasurement();
            if (_sfaActive1) {
                Serial.println("    ✓ SFA30 #1 initialized");
                break;
            }
        }
        if (attempt < 3) {
            Serial.print("    Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
            recoverMainI2CBus();
        } else {
            Serial.println("    ✗ SFA30 #1 not detected");
        }
    }
    
    // SFA30 #2 on alternate bus (25/26)
    Serial.println("  SFA30 #2: Alternate bus (SDA=25, SCL=26)");
    for (int attempt = 1; attempt <= 3; attempt++) {
        _sfaActive2 = _sfaSensor2.begin();
        if (_sfaActive2) {
            _sfaActive2 = _sfaSensor2.startContinuousMeasurement();
            if (_sfaActive2) {
                Serial.println("    ✓ SFA30 #2 initialized");
                break;
            }
        }
        if (attempt < 3) {
            Serial.print("    Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
            recoverAltI2CBus();
        } else {
            Serial.println("    ✗ SFA30 #2 not detected");
        }
    }
    
    // Initialize SGP41 sensors
    Serial.println("\n[SGP41] Initializing...");
    
    // SGP41 #1 on main bus (21/22)
    Serial.println("  SGP41 #1: Main bus (SDA=21, SCL=22)");
    for (int attempt = 1; attempt <= 3; attempt++) {
        _sgpActive1 = _sgpSensor1.begin();
        if (_sgpActive1) {
            Serial.println("    ✓ SGP41 #1 initialized");
            break;
        }
        if (attempt < 3) {
            Serial.print("    Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
            recoverMainI2CBus();
        } else {
            Serial.println("    ✗ SGP41 #1 not detected");
        }
    }
    
    // SGP41 #2 on alternate bus (25/26)
    Serial.println("  SGP41 #2: Alternate bus (SDA=25, SCL=26)");
    for (int attempt = 1; attempt <= 3; attempt++) {
        _sgpActive2 = _sgpSensor2.begin();
        if (_sgpActive2) {
            Serial.println("    ✓ SGP41 #2 initialized");
            break;
        }
        if (attempt < 3) {
            Serial.print("    Attempt "); Serial.print(attempt); Serial.println(" failed, retrying...");
            delay(100);
            recoverAltI2CBus();
        } else {
            Serial.println("    ✗ SGP41 #2 not detected");
        }
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

void SensorManager::recoverMainI2CBus() {
    unsigned long now = millis();
    if (now - _lastMainBusRecovery < BUS_RECOVERY_INTERVAL) {
        return;
    }
    _lastMainBusRecovery = now;
    
    Serial.println("[Main I2C] Checking bus...");
    
    pinMode(_i2cConfig.mainSDA, INPUT_PULLUP);
    pinMode(_i2cConfig.mainSCL, INPUT_PULLUP);
    delay(10);
    
    bool sdaStuck = digitalRead(_i2cConfig.mainSDA) == LOW;
    bool sclStuck = digitalRead(_i2cConfig.mainSCL) == LOW;
    
    if (sdaStuck || sclStuck) {
        Serial.println("  Main I2C bus appears stuck. Attempting recovery...");
        
        pinMode(_i2cConfig.mainSDA, OUTPUT);
        digitalWrite(_i2cConfig.mainSDA, HIGH);
        pinMode(_i2cConfig.mainSCL, OUTPUT);
        
        // Generate clock pulses
        for (int i = 0; i < 10; i++) {
            digitalWrite(_i2cConfig.mainSCL, LOW);
            delayMicroseconds(5);
            digitalWrite(_i2cConfig.mainSCL, HIGH);
            delayMicroseconds(5);
        }
        
        // Generate STOP condition
        digitalWrite(_i2cConfig.mainSDA, LOW);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.mainSCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.mainSDA, HIGH);
        delayMicroseconds(5);
        
        pinMode(_i2cConfig.mainSDA, INPUT_PULLUP);
        pinMode(_i2cConfig.mainSCL, INPUT_PULLUP);
        
        Serial.println("  Main I2C bus recovery complete");
    } else {
        Serial.println("  Main I2C bus appears free");
    }
    
    delay(50);
}

void SensorManager::recoverAltI2CBus() {
    unsigned long now = millis();
    if (now - _lastAltBusRecovery < BUS_RECOVERY_INTERVAL) {
        return;
    }
    _lastAltBusRecovery = now;
    
    Serial.println("[Alt I2C] Checking bus...");
    
    pinMode(_i2cConfig.altSDA, INPUT_PULLUP);
    pinMode(_i2cConfig.altSCL, INPUT_PULLUP);
    delay(10);
    
    bool sdaStuck = digitalRead(_i2cConfig.altSDA) == LOW;
    bool sclStuck = digitalRead(_i2cConfig.altSCL) == LOW;
    
    if (sdaStuck || sclStuck) {
        Serial.println("  Alt I2C bus appears stuck. Attempting recovery...");
        
        pinMode(_i2cConfig.altSDA, OUTPUT);
        digitalWrite(_i2cConfig.altSDA, HIGH);
        pinMode(_i2cConfig.altSCL, OUTPUT);
        
        // Generate clock pulses
        for (int i = 0; i < 10; i++) {
            digitalWrite(_i2cConfig.altSCL, LOW);
            delayMicroseconds(5);
            digitalWrite(_i2cConfig.altSCL, HIGH);
            delayMicroseconds(5);
        }
        
        // Generate STOP condition
        digitalWrite(_i2cConfig.altSDA, LOW);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.altSCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(_i2cConfig.altSDA, HIGH);
        delayMicroseconds(5);
        
        pinMode(_i2cConfig.altSDA, INPUT_PULLUP);
        pinMode(_i2cConfig.altSCL, INPUT_PULLUP);
        
        Serial.println("  Alt I2C bus recovery complete");
    } else {
        Serial.println("  Alt I2C bus appears free");
    }
    
    delay(50);
}

bool SensorManager::readSDP() {
    if (!_sdpActive) {
        // Try to recover SDP810
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SDP810] Attempting recovery...");
            recoverMainI2CBus();
            _sdpActive = _sdpSensor.begin();
            if (_sdpActive) {
                _sdpActive = _sdpSensor.startContinuousMeasurement();
                if (_sdpActive) {
                    Serial.println("[SDP810] ✓ Recovery successful");
                }
            }
        }
        return false;
    }
    
    bool success = _sdpSensor.readMeasurement(_sdpData);
    
    // If reading fails, try to recover
    static int failCount = 0;
    if (!success) {
        failCount++;
        if (failCount > 5) {
            Serial.println("[SDP810] Multiple read failures, attempting recovery...");
            
            _sdpSensor.stopContinuousMeasurement();
            delay(50);
            recoverMainI2CBus();
            _sdpActive = _sdpSensor.startContinuousMeasurement();
            delay(50);
            
            if (_sdpActive) {
                success = _sdpSensor.readMeasurement(_sdpData);
                if (success) {
                    Serial.println("[SDP810] ✓ Recovery successful");
                    failCount = 0;
                } else {
                    Serial.println("[SDP810] ✗ Recovery failed");
                    _sdpActive = false;
                }
            }
        }
    } else {
        failCount = 0;
    }
    
    return success;
}

bool SensorManager::readPMS1() {
    if (!_pmsActive1) {
        _pmsData1.valid = false;
        
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[PMS5003 #1] Attempting recovery...");
            _pmsActive1 = _pmsSensor1.begin();
        }
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
        
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[PMS5003 #2] Attempting recovery...");
            _pmsActive2 = _pmsSensor2.begin();
        }
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
    if (!_scdActive) {
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SCD40] Attempting recovery...");
            recoverMainI2CBus();
            _scdActive = _scdSensor.begin();
            if (_scdActive) {
                _scdActive = _scdSensor.startPeriodicMeasurement();
                if (_scdActive) {
                    Serial.println("[SCD40] ✓ Recovery successful");
                }
            }
        }
        return false;
    }
    
    bool success = _scdSensor.readMeasurement(_scdData);
    
    static int failCount = 0;
    if (!success) {
        failCount++;
        if (failCount > 5) {
            Serial.println("[SCD40] Multiple read failures, marking as inactive");
            _scdActive = false;
            failCount = 0;
        }
    } else {
        failCount = 0;
    }
    
    return success;
}

bool SensorManager::readSFA1() {
    if (!_sfaActive1) {
        // Try to recover if disconnected
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SFA30 #1] Attempting recovery...");
            recoverMainI2CBus();
            _sfaActive1 = _sfaSensor1.begin();
            if (_sfaActive1) {
                _sfaActive1 = _sfaSensor1.startContinuousMeasurement();
                if (_sfaActive1) {
                    Serial.println("[SFA30 #1] ✓ Recovery successful");
                }
            }
        }
        return false;
    }
    
    bool result = _sfaSensor1.readMeasurement(_sfaData1);
    
    // CRITICAL FIX: Only validate for sensor errors, NOT for high readings
    if (result && _sfaData1.valid) {
        bool dataValid = true;
        
        // Only check for SENSOR ERRORS, not high readings!
        
        // 1. Check for negative values (sensor error)
        if (_sfaData1.formaldehyde < 0) {
            dataValid = false;
            Serial.println("[SFA30 #1] ERROR: Negative formaldehyde reading: " + String(_sfaData1.formaldehyde) + " ppb");
        }
        
        // 2. Check for impossibly high sensor saturation (>10,000 ppb is likely sensor failure)
        if (_sfaData1.formaldehyde > 10000) {
            dataValid = false;
            Serial.println("[SFA30 #1] ERROR: Sensor saturated/failed: " + String(_sfaData1.formaldehyde) + " ppb");
        }
        
        // 3. Temperature range check (sensor spec: -40°C to +125°C)
        if (_sfaData1.temperature < -40 || _sfaData1.temperature > 125) {
            dataValid = false;
            Serial.println("[SFA30 #1] ERROR: Temperature out of sensor range: " + String(_sfaData1.temperature) + " °C");
        }
        
        // 4. Humidity range check (0-100% RH)
        if (_sfaData1.humidity < 0 || _sfaData1.humidity > 100) {
            dataValid = false;
            Serial.println("[SFA30 #1] ERROR: Humidity out of range: " + String(_sfaData1.humidity) + " %");
        }
        
        // 5. Check for all zeros (disconnected sensor)
        if (_sfaData1.formaldehyde == 0.0 && _sfaData1.temperature == 0.0 && _sfaData1.humidity == 0.0) {
            dataValid = false;
            Serial.println("[SFA30 #1] ERROR: All readings are zero - sensor may be disconnected");
        }
        
        // REMOVED: No continuous warnings - only show in periodic display
        // Data is valid, just keep reading silently
        
        if (!dataValid) {
            _sfaData1.valid = false;
            result = false;
        }
    }
    
    // If reading fails consistently, try bus recovery
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 3) {
            Serial.println("[SFA30 #1] Multiple failures, attempting bus recovery...");
            recoverMainI2CBus();
            failCount = 0;
            
            // REMOVED: Don't mark as inactive for too many failures
            // Just keep trying!
        }
    } else {
        failCount = 0;
    }
    
    return result;
}

bool SensorManager::readSFA2() {
    if (!_sfaActive2) {
        // Try to recover if disconnected
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SFA30 #2] Attempting recovery...");
            recoverAltI2CBus();
            _sfaActive2 = _sfaSensor2.begin();
            if (_sfaActive2) {
                _sfaActive2 = _sfaSensor2.startContinuousMeasurement();
                if (_sfaActive2) {
                    Serial.println("[SFA30 #2] ✓ Recovery successful");
                }
            }
        }
        return false;
    }
    
    bool result = _sfaSensor2.readMeasurement(_sfaData2);
    
    // CRITICAL FIX: Only validate for sensor errors, NOT for high readings
    if (result && _sfaData2.valid) {
        bool dataValid = true;
        
        // Only check for SENSOR ERRORS, not high readings!
        
        // 1. Check for negative values (sensor error)
        if (_sfaData2.formaldehyde < 0) {
            dataValid = false;
            Serial.println("[SFA30 #2] ERROR: Negative formaldehyde reading: " + String(_sfaData2.formaldehyde) + " ppb");
        }
        
        // 2. Check for impossibly high sensor saturation (>10,000 ppb is likely sensor failure)
        if (_sfaData2.formaldehyde > 10000) {
            dataValid = false;
            Serial.println("[SFA30 #2] ERROR: Sensor saturated/failed: " + String(_sfaData2.formaldehyde) + " ppb");
        }
        
        // 3. Temperature range check (sensor spec: -40°C to +125°C)
        if (_sfaData2.temperature < -40 || _sfaData2.temperature > 125) {
            dataValid = false;
            Serial.println("[SFA30 #2] ERROR: Temperature out of sensor range: " + String(_sfaData2.temperature) + " °C");
        }
        
        // 4. Humidity range check (0-100% RH)
        if (_sfaData2.humidity < 0 || _sfaData2.humidity > 100) {
            dataValid = false;
            Serial.println("[SFA30 #2] ERROR: Humidity out of range: " + String(_sfaData2.humidity) + " %");
        }
        
        // 5. Check for all zeros (disconnected sensor)
        if (_sfaData2.formaldehyde == 0.0 && _sfaData2.temperature == 0.0 && _sfaData2.humidity == 0.0) {
            dataValid = false;
            Serial.println("[SFA30 #2] ERROR: All readings are zero - sensor may be disconnected");
        }
        
        // REMOVED: No continuous warnings - only show in periodic display
        // Data is valid, just keep reading silently
        
        if (!dataValid) {
            _sfaData2.valid = false;
            result = false;
        }
    }
    
    // If reading fails consistently, try bus recovery
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 3) {
            Serial.println("[SFA30 #2] Multiple failures, attempting bus recovery...");
            recoverAltI2CBus();
            failCount = 0;
            
            // REMOVED: Don't mark as inactive for too many failures
            // Just keep trying!
        }
    } else {
        failCount = 0;
    }
    
    return result;
}

bool SensorManager::readSGP1() {
    if (!_sgpActive1) {
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SGP41 #1] Attempting recovery...");
            recoverMainI2CBus();
            _sgpActive1 = _sgpSensor1.begin();
            if (_sgpActive1) {
                Serial.println("[SGP41 #1] ✓ Recovery successful");
            }
        }
        return false;
    }
    
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    bool result = _sgpSensor1.measureRawSignals(_sgpData1, temp, hum);
    
    // If reading fails consistently, try bus recovery
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 3) {
            Serial.println("[SGP41 #1] Multiple failures, attempting bus recovery...");
            recoverMainI2CBus();
            failCount = 0;
        }
    } else {
        failCount = 0;
    }
    
    return result;
}

bool SensorManager::readSGP2() {
    if (!_sgpActive2) {
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SGP41 #2] Attempting recovery...");
            recoverAltI2CBus();
            _sgpActive2 = _sgpSensor2.begin();
            if (_sgpActive2) {
                Serial.println("[SGP41 #2] ✓ Recovery successful");
            }
        }
        return false;
    }
    
    float temp = getAverageTemperature();
    float hum = getAverageHumidity();
    
    bool result = _sgpSensor2.measureRawSignals(_sgpData2, temp, hum);
    
    // If reading fails consistently, try bus recovery
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 3) {
            Serial.println("[SGP41 #2] Multiple failures, attempting bus recovery...");
            recoverAltI2CBus();
            failCount = 0;
        }
    } else {
        failCount = 0;
    }
    
    return result;
}

bool SensorManager::readSHT1() {
    if (!_shtActive1) {
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SHT31 #1] Attempting recovery...");
            recoverMainI2CBus();
            _shtActive1 = _shtSensor1.begin();
            if (_shtActive1) {
                Serial.println("[SHT31 #1] ✓ Recovery successful");
            }
        }
        return false;
    }
    
    bool result = _shtSensor1.readMeasurement(_shtData1);
    
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 5) {
            Serial.println("[SHT31 #1] Multiple read failures, marking as inactive");
            _shtActive1 = false;
            failCount = 0;
        }
    } else {
        failCount = 0;
    }
    
    return result;
}

bool SensorManager::readSHT2() {
    if (!_shtActive2) {
        // Try to recover
        static unsigned long lastRecoveryAttempt = 0;
        unsigned long now = millis();
        
        if (now - lastRecoveryAttempt > 30000) {
            lastRecoveryAttempt = now;
            Serial.println("[SHT31 #2] Attempting recovery...");
            recoverMainI2CBus();
            _shtActive2 = _shtSensor2.begin();
            if (_shtActive2) {
                Serial.println("[SHT31 #2] ✓ Recovery successful");
            }
        }
        return false;
    }
    
    bool result = _shtSensor2.readMeasurement(_shtData2);
    
    static int failCount = 0;
    if (!result) {
        failCount++;
        if (failCount > 5) {
            Serial.println("[SHT31 #2] Multiple read failures, marking as inactive");
            _shtActive2 = false;
            failCount = 0;
        }
    } else {
        failCount = 0;
    }
    
    return result;
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