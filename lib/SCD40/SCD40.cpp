#include "SCD40.h"
#include <math.h>

// Command definitions
const uint16_t SCD40::CMD_START_PERIODIC_MEAS = 0x21B1;
const uint16_t SCD40::CMD_STOP_PERIODIC_MEAS = 0x3F86;
const uint16_t SCD40::CMD_READ_MEAS = 0xEC05;
const uint16_t SCD40::CMD_DATA_READY = 0xE4B8;
const uint16_t SCD40::CMD_SINGLE_SHOT_MEAS = 0x219D;
const uint16_t SCD40::CMD_SINGLE_SHOT_RHT_ONLY = 0x2196;
const uint16_t SCD40::CMD_SET_TEMP_OFFSET = 0x241D;
const uint16_t SCD40::CMD_GET_TEMP_OFFSET = 0x2318;
const uint16_t SCD40::CMD_SET_ALTITUDE = 0x2427;
const uint16_t SCD40::CMD_GET_ALTITUDE = 0x2322;
const uint16_t SCD40::CMD_SET_AMBIENT_PRESSURE = 0xE000;
const uint16_t SCD40::CMD_FORCED_RECAL = 0x362F;
const uint16_t SCD40::CMD_GET_FORCED_RECAL = 0x2313;
const uint16_t SCD40::CMD_SET_AUTO_CALIB = 0x2416;
const uint16_t SCD40::CMD_GET_AUTO_CALIB = 0x2313;
const uint16_t SCD40::CMD_PERSIST_SETTINGS = 0x3615;
const uint16_t SCD40::CMD_GET_SERIAL = 0x3682;
const uint16_t SCD40::CMD_GET_FIRMWARE = 0x202F;
const uint16_t SCD40::CMD_PERFORM_SELF_TEST = 0x3639;
const uint16_t SCD40::CMD_REINIT = 0x3646;
const uint16_t SCD40::CMD_SOFT_RESET = 0x0006;
const uint16_t SCD40::CMD_LOW_POWER_PERIODIC_MEAS = 0x21AC;

SCD40::SCD40(TwoWire& wire, uint8_t address) 
    : _wire(wire), _address(address), _measurementActive(false), _currentMode(CONTINUOUS) {}

bool SCD40::begin() {
    _wire.begin();
    delay(100);
    
    // Check if sensor is connected
    if (!isConnected()) {
        return false;
    }
    
    // Perform soft reset
    if (!softReset()) {
        return false;
    }
    
    delay(2000); // Wait for sensor to initialize after reset
    
    return true;
}

bool SCD40::softReset() {
    return _writeCommand(CMD_SOFT_RESET);
}

bool SCD40::startPeriodicMeasurement(uint16_t interval_seconds) {
    if (interval_seconds < 5) interval_seconds = 5;
    
    _currentMode = CONTINUOUS;
    _measurementActive = true;
    
    return _writeCommand(CMD_START_PERIODIC_MEAS);
}

bool SCD40::startLowPowerPeriodicMeasurement(uint16_t interval_seconds) {
    if (interval_seconds < 30) interval_seconds = 30;
    
    _currentMode = LOW_POWER_CONTINUOUS;
    _measurementActive = true;
    
    return _writeCommand(CMD_LOW_POWER_PERIODIC_MEAS);
}

bool SCD40::stopPeriodicMeasurement() {
    _measurementActive = false;
    return _writeCommand(CMD_STOP_PERIODIC_MEAS);
}

bool SCD40::measureSingleShot() {
    _currentMode = SINGLE_SHOT;
    return _writeCommand(CMD_SINGLE_SHOT_MEAS);
}

bool SCD40::measureSingleShotRHTOnly() {
    return _writeCommand(CMD_SINGLE_SHOT_RHT_ONLY);
}

bool SCD40::readMeasurement(Data& data) {
    if (!_measurementActive && _currentMode != SINGLE_SHOT) {
        return false;
    }
    
    uint8_t buffer[9]; // 3 measurements * (2 bytes + 1 CRC)
    
    if (!_writeCommand(CMD_READ_MEAS)) {
        return false;
    }
    
    delay(1); // Small delay
    
    if (!_readResponse(buffer, 9)) {
        return false;
    }
    
    // Check CRCs for each measurement
    for (int i = 0; i < 3; i++) {
        if (!_checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return false;
        }
    }
    
    data.raw_co2 = (buffer[0] << 8) | buffer[1];
    data.raw_temp = (buffer[3] << 8) | buffer[4];
    data.raw_humidity = (buffer[6] << 8) | buffer[7];
    
    data.co2 = _convertCO2(data.raw_co2);
    data.temperature = _convertTemperature(data.raw_temp);
    data.humidity = _convertHumidity(data.raw_humidity);
    
    return true;
}

bool SCD40::isDataReady() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_DATA_READY)) {
        return false;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return false;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return false;
    }
    
    uint16_t ready = (buffer[0] << 8) | buffer[1];
    return (ready & 0x07FF) != 0; // Check if data is ready
}

bool SCD40::setForcedRecalibration(float co2_reference) {
    if (co2_reference < 400 || co2_reference > 2000) {
        return false;
    }
    
    uint16_t ref = (uint16_t)co2_reference;
    return _writeCommandWithData(CMD_FORCED_RECAL, &ref, 1);
}

float SCD40::getForcedRecalibration() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_FORCED_RECAL)) {
        return 0.0f;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return 0.0f;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return 0.0f;
    }
    
    uint16_t frc_correction = (buffer[0] << 8) | buffer[1];
    return (float)frc_correction;
}

bool SCD40::setAutomaticSelfCalibration(bool enabled) {
    uint16_t value = enabled ? 1 : 0;
    return _writeCommandWithData(CMD_SET_AUTO_CALIB, &value, 1);
}

bool SCD40::getAutomaticSelfCalibration() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_AUTO_CALIB)) {
        return false;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return false;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return false;
    }
    
    uint16_t asc_status = (buffer[0] << 8) | buffer[1];
    return (asc_status == 1);
}

bool SCD40::setTemperatureOffset(float offset) {
    if (offset < -10.0f || offset > 10.0f) {
        return false;
    }
    
    uint16_t offset_raw = (uint16_t)(offset * 100.0f);
    return _writeCommandWithData(CMD_SET_TEMP_OFFSET, &offset_raw, 1);
}

float SCD40::getTemperatureOffset() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_TEMP_OFFSET)) {
        return 0.0f;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return 0.0f;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return 0.0f;
    }
    
    uint16_t offset_raw = (buffer[0] << 8) | buffer[1];
    return offset_raw / 100.0f;
}

bool SCD40::setSensorAltitude(uint16_t altitude) {
    return _writeCommandWithData(CMD_SET_ALTITUDE, &altitude, 1);
}

uint16_t SCD40::getSensorAltitude() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_ALTITUDE)) {
        return 0;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return 0;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return 0;
    }
    
    return (buffer[0] << 8) | buffer[1];
}

bool SCD40::setAmbientPressure(uint16_t pressure) {
    if (pressure < 70000 || pressure > 120000) {
        return false; // Pressure in Pa
    }
    
    return _writeCommandWithData(CMD_SET_AMBIENT_PRESSURE, &pressure, 1);
}

bool SCD40::persistSettings() {
    return _writeCommand(CMD_PERSIST_SETTINGS);
}

bool SCD40::performFactoryReset() {
    return _writeCommand(CMD_SOFT_RESET); // Same as soft reset for SCD40
}

bool SCD40::performSelfTest() {
    if (!_writeCommand(CMD_PERFORM_SELF_TEST)) {
        return false;
    }
    
    delay(10000); // Self-test takes up to 10 seconds
    
    uint8_t buffer[3];
    if (!_readResponse(buffer, 3)) {
        return false;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return false;
    }
    
    uint16_t result = (buffer[0] << 8) | buffer[1];
    return (result == 0); // 0 = no malfunction detected
}

bool SCD40::reinit() {
    return _writeCommand(CMD_REINIT);
}

uint16_t SCD40::getSerialNumber(uint16_t* serial, uint8_t max_serial) {
    if (max_serial < 3) return 0;
    
    uint8_t buffer[9];
    
    if (!_writeCommand(CMD_GET_SERIAL)) {
        return 0;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 9)) {
        return 0;
    }
    
    // Check CRCs
    for (int i = 0; i < 3; i++) {
        if (!_checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return 0;
        }
    }
    
    serial[0] = (buffer[0] << 8) | buffer[1];
    serial[1] = (buffer[3] << 8) | buffer[4];
    serial[2] = (buffer[6] << 8) | buffer[7];
    
    return 3;
}

String SCD40::getSerialNumberString() {
    uint16_t serial[3];
    
    if (getSerialNumber(serial, 3) != 3) {
        return "Unknown";
    }
    
    char buffer[13]; // 12 hex digits + null
    sprintf(buffer, "%04X%04X%04X", serial[0], serial[1], serial[2]);
    return String(buffer);
}

uint16_t SCD40::getFirmwareVersion() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_FIRMWARE)) {
        return 0;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 3)) {
        return 0;
    }
    
    if (!_checkCRC(buffer, 2, buffer[2])) {
        return 0;
    }
    
    return (buffer[0] << 8) | buffer[1];
}

bool SCD40::getFeatures(uint16_t* features) {
    uint8_t buffer[6];
    
    if (!_writeCommand(CMD_GET_FIRMWARE)) {
        return false;
    }
    
    delay(1);
    
    if (!_readResponse(buffer, 6)) {
        return false;
    }
    
    // Check CRCs
    for (int i = 0; i < 2; i++) {
        if (!_checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return false;
        }
    }
    
    features[0] = (buffer[0] << 8) | buffer[1];
    features[1] = (buffer[3] << 8) | buffer[4];
    
    return true;
}

bool SCD40::isConnected() {
    _wire.beginTransmission(_address);
    return (_wire.endTransmission() == 0);
}

bool SCD40::isMeasuring() {
    return _measurementActive;
}

String SCD40::getCO2QualityLabel(float co2_ppm) {
    if (co2_ppm < 800) return "Excellent";
    else if (co2_ppm < 1000) return "Good";
    else if (co2_ppm < 1200) return "Moderate";
    else if (co2_ppm < 1500) return "Poor";
    else return "Unhealthy";
}

String SCD40::getCO2HealthImpact(float co2_ppm) {
    if (co2_ppm < 800) return "Optimal indoor air";
    else if (co2_ppm < 1000) return "Slight drowsiness possible";
    else if (co2_ppm < 1200) return "Reduced concentration";
    else if (co2_ppm < 1500) return "Headaches, sleepiness";
    else return "Significant health risks";
}

float SCD40::calculateDewPoint(float temperature, float humidity) {
    // Magnus formula for dew point calculation
    float a = 17.27;
    float b = 237.7;
    
    float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

float SCD40::calculateAbsoluteHumidity(float temperature, float humidity) {
    // Calculate absolute humidity in g/m³
    float saturationVaporPressure = 6.112 * exp((17.67 * temperature) / (temperature + 243.5));
    float vaporPressure = (humidity / 100.0) * saturationVaporPressure;
    return (216.7 * vaporPressure) / (temperature + 273.15);
}

// Private methods

bool SCD40::_writeCommand(uint16_t command) {
    _wire.beginTransmission(_address);
    _wire.write(command >> 8);
    _wire.write(command & 0xFF);
    return (_wire.endTransmission() == 0);
}

bool SCD40::_writeCommandWithData(uint16_t command, const uint16_t* data, uint8_t data_words) {
    uint8_t buffer[2 + data_words * 3]; // Command + data words with CRC
    
    buffer[0] = command >> 8;
    buffer[1] = command & 0xFF;
    
    for (uint8_t i = 0; i < data_words; i++) {
        buffer[2 + i*3] = data[i] >> 8;
        buffer[3 + i*3] = data[i] & 0xFF;
        buffer[4 + i*3] = _crc8(&buffer[2 + i*3], 2);
    }
    
    _wire.beginTransmission(_address);
    _wire.write(buffer, sizeof(buffer));
    return (_wire.endTransmission() == 0);
}

bool SCD40::_readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms) {
    delay(delay_ms);
    
    _wire.requestFrom(_address, length);
    
    if (_wire.available() < length) {
        return false;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = _wire.read();
    }
    
    return true;
}

bool SCD40::_readResponseWithCRC(uint16_t* data, uint8_t data_words) {
    uint8_t buffer[data_words * 3];
    
    if (!_readResponse(buffer, data_words * 3)) {
        return false;
    }
    
    for (uint8_t i = 0; i < data_words; i++) {
        if (!_checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return false;
        }
        data[i] = (buffer[i*3] << 8) | buffer[i*3 + 1];
    }
    
    return true;
}

uint8_t SCD40::_crc8(uint8_t data[], uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool SCD40::_checkCRC(uint8_t data[], uint8_t len, uint8_t checksum) {
    return (_crc8(data, len) == checksum);
}

float SCD40::_convertCO2(uint16_t raw) {
    return (float)raw; // Direct conversion for SCD40
}

float SCD40::_convertTemperature(uint16_t raw) {
    return -45.0f + 175.0f * (float)raw / 65535.0f;
}

float SCD40::_convertHumidity(uint16_t raw) {
    return 100.0f * (float)raw / 65535.0f;
}