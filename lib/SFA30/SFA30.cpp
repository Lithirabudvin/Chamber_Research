#include "SFA30.h"

// Initialize static constants HERE in the .cpp file
const uint16_t SFA30::CMD_START_CONT_MEAS = 0x0006;
const uint16_t SFA30::CMD_STOP_MEAS = 0x0104;
const uint16_t SFA30::CMD_READ_MEAS = 0x0326;
const uint16_t SFA30::CMD_SINGLE_MEAS = 0x2403;
const uint16_t SFA30::CMD_SOFT_RESET = 0xD304;
const uint16_t SFA30::CMD_GET_SERIAL = 0xD033;
const uint16_t SFA30::CMD_GET_FEATURES = 0x202F;
const uint16_t SFA30::CMD_SET_INTERVAL = 0x4600;
const uint16_t SFA30::CMD_GET_INTERVAL = 0x4601;
const uint16_t SFA30::CMD_SET_TEMP_OFFSET = 0x6004;
const uint16_t SFA30::CMD_GET_TEMP_OFFSET = 0x6005;
const uint16_t SFA30::CMD_START_FAN_CLEAN = 0x5607;
const uint16_t SFA30::CMD_GET_FAN_CLEAN = 0x5608;

SFA30::SFA30(TwoWire& wire, uint8_t address) 
    : _wire(wire), _address(address), _mode(CONTINUOUS), _measurementActive(false) {}

bool SFA30::begin() {
    _wire.begin();
    delay(100);
    
    // Try to communicate with sensor
    if (!softReset()) {
        return false;
    }
    
    delay(50); // Wait after reset
    
    // Check if sensor responds
    return isConnected();
}

bool SFA30::softReset() {
    return _writeCommand(CMD_SOFT_RESET);
}

bool SFA30::startContinuousMeasurement(MeasurementMode mode) {
    _mode = mode;
    
    uint8_t mode_data[1] = {static_cast<uint8_t>(mode)};
    
    if (!_writeCommandWithData(CMD_START_CONT_MEAS, mode_data, 1)) {
        return false;
    }
    
    _measurementActive = true;
    delay(10); // Small delay after starting measurement
    
    return true;
}

bool SFA30::stopMeasurement() {
    if (!_writeCommand(CMD_STOP_MEAS)) {
        return false;
    }
    
    _measurementActive = false;
    return true;
}

bool SFA30::readMeasurement(Data& data) {
    if (!_measurementActive) {
        return false;
    }
    
    uint16_t hcho_raw, rh_raw, temp_raw;
    
    if (!_readMeasurementRaw(hcho_raw, rh_raw, temp_raw)) {
        return false;
    }
    
    data.raw_hcho = hcho_raw;
    data.raw_rh = rh_raw;
    data.raw_temp = temp_raw;
    data.formaldehyde = convertFormaldehyde(hcho_raw);
    data.humidity = convertHumidity(rh_raw);
    data.temperature = convertTemperature(temp_raw);
    
    return true;
}

bool SFA30::triggerSingleMeasurement(Data& data) {
    if (!_writeCommand(CMD_SINGLE_MEAS)) {
        return false;
    }
    
    delay(250); // Single measurement takes ~250ms
    
    return readMeasurement(data);
}

bool SFA30::setMeasurementInterval(uint16_t interval_seconds) {
    uint8_t data[2];
    data[0] = (interval_seconds >> 8) & 0xFF;
    data[1] = interval_seconds & 0xFF;
    
    return _writeCommandWithData(CMD_SET_INTERVAL, data, 2);
}

uint16_t SFA30::getMeasurementInterval() {
    uint8_t buffer[3]; // 2 bytes data + 1 byte CRC
    
    if (!_writeCommand(CMD_GET_INTERVAL)) {
        return 0;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 3)) {
        return 0;
    }
    
    if (!_checkCrc(buffer, 2, buffer[2])) {
        return 0;
    }
    
    return (buffer[0] << 8) | buffer[1];
}

bool SFA30::setTemperatureOffset(float offset) {
    // Convert from float to raw value (scale factor from datasheet)
    int16_t offset_raw = static_cast<int16_t>(offset * 200.0f);
    
    uint8_t data[2];
    data[0] = (offset_raw >> 8) & 0xFF;
    data[1] = offset_raw & 0xFF;
    
    return _writeCommandWithData(CMD_SET_TEMP_OFFSET, data, 2);
}

float SFA30::getTemperatureOffset() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_TEMP_OFFSET)) {
        return 0.0f;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 3)) {
        return 0.0f;
    }
    
    if (!_checkCrc(buffer, 2, buffer[2])) {
        return 0.0f;
    }
    
    int16_t offset_raw = (buffer[0] << 8) | buffer[1];
    return offset_raw / 200.0f;
}

bool SFA30::startFanCleaning() {
    return _writeCommand(CMD_START_FAN_CLEAN);
}

bool SFA30::isFanCleaningActive() {
    uint8_t buffer[3];
    
    if (!_writeCommand(CMD_GET_FAN_CLEAN)) {
        return false;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 3)) {
        return false;
    }
    
    if (!_checkCrc(buffer, 2, buffer[2])) {
        return false;
    }
    
    return (buffer[1] != 0); // Cleaning active if second byte != 0
}

uint32_t SFA30::getSerialNumber() {
    uint8_t buffer[9]; // 3 words (6 bytes) + 3 CRC bytes
    
    if (!_writeCommand(CMD_GET_SERIAL)) {
        return 0;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 9)) {
        return 0;
    }
    
    // Check CRCs for each word
    for (int i = 0; i < 3; i++) {
        if (!_checkCrc(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return 0;
        }
    }
    
    // Combine words to form serial number
    uint32_t serial = 0;
    for (int i = 0; i < 3; i++) {
        serial = (serial << 16) | ((buffer[i*3] << 8) | buffer[i*3 + 1]);
    }
    
    return serial;
}

String SFA30::getSerialNumberString() {
    uint32_t serial = getSerialNumber();
    if (serial == 0) {
        return "Unknown";
    }
    
    char buffer[13]; // 12 hex digits + null terminator
    sprintf(buffer, "%08lX", serial);
    return String(buffer);
}

uint8_t SFA30::getProductType() {
    uint8_t buffer[6]; // 2 words (4 bytes) + 2 CRC bytes
    
    if (!_writeCommand(CMD_GET_FEATURES)) {
        return 0;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 6)) {
        return 0;
    }
    
    // Check CRCs
    for (int i = 0; i < 2; i++) {
        if (!_checkCrc(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return 0;
        }
    }
    
    return buffer[0]; // First byte is product type
}

uint8_t SFA30::getProductVersion() {
    uint8_t buffer[6];
    
    if (!_writeCommand(CMD_GET_FEATURES)) {
        return 0;
    }
    
    delay(10);
    
    if (!_readResponse(buffer, 6)) {
        return 0;
    }
    
    // Check CRCs
    for (int i = 0; i < 2; i++) {
        if (!_checkCrc(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return 0;
        }
    }
    
    return buffer[1]; // Second byte is product version
}

bool SFA30::isConnected() {
    _wire.beginTransmission(_address);
    return (_wire.endTransmission() == 0);
}

bool SFA30::isDataReady() {
    // For SFA30, we can try to read measurement
    // If it fails with NACK, data might not be ready
    uint8_t buffer[9];
    
    _wire.beginTransmission(_address);
    _wire.write(CMD_READ_MEAS >> 8);
    _wire.write(CMD_READ_MEAS & 0xFF);
    
    if (_wire.endTransmission(false) != 0) { // Don't send stop condition
        return false;
    }
    
    delay(1); // Small delay
    
    // Request 1 byte to check if sensor responds
    _wire.requestFrom(_address, (uint8_t)1);
    return (_wire.available() > 0);
}

bool SFA30::hasError() {
    // Simple error check - try to read serial number
    return (getSerialNumber() == 0);
}

float SFA30::convertFormaldehyde(uint16_t raw) {
    // According to datasheet: HCHO = (raw - 0x8000) / 5.0
    int16_t signed_raw = static_cast<int16_t>(raw);
    return (signed_raw - 0x8000) / 5.0f;
}

float SFA30::convertHumidity(uint16_t raw) {
    // RH = raw / 100.0
    return raw / 100.0f;
}

float SFA30::convertTemperature(uint16_t raw) {
    // Temperature = raw / 200.0
    return raw / 200.0f;
}

String SFA30::getAirQualityLabel(float hcho_ppm) {
    // WHO guidelines for formaldehyde:
    // < 0.08 ppm: Good
    // 0.08-0.1 ppm: Moderate
    // > 0.1 ppm: Poor
    
    if (hcho_ppm < 0.08f) {
        return "Good";
    } else if (hcho_ppm < 0.1f) {
        return "Moderate";
    } else if (hcho_ppm < 0.2f) {
        return "Poor";
    } else {
        return "Hazardous";
    }
}

// Private methods

bool SFA30::_writeCommand(uint16_t command) {
    _wire.beginTransmission(_address);
    _wire.write(command >> 8);    // MSB
    _wire.write(command & 0xFF);  // LSB
    return (_wire.endTransmission() == 0);
}

bool SFA30::_writeCommandWithData(uint16_t command, const uint8_t* data, uint8_t data_len) {
    _wire.beginTransmission(_address);
    _wire.write(command >> 8);
    _wire.write(command & 0xFF);
    
    for (uint8_t i = 0; i < data_len; i++) {
        _wire.write(data[i]);
    }
    
    return (_wire.endTransmission() == 0);
}

bool SFA30::_readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms) {
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

bool SFA30::_readMeasurementRaw(uint16_t& hcho, uint16_t& rh, uint16_t& temp) {
    uint8_t buffer[9]; // 3 measurements * (2 bytes + 1 CRC)
    
    if (!_writeCommand(CMD_READ_MEAS)) {
        return false;
    }
    
    delay(10); // Measurement read delay
    
    if (!_readResponse(buffer, 9)) {
        return false;
    }
    
    // Check CRC for each measurement
    for (int i = 0; i < 3; i++) {
        if (!_checkCrc(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return false;
        }
    }
    
    hcho = (buffer[0] << 8) | buffer[1];
    rh = (buffer[3] << 8) | buffer[4];
    temp = (buffer[6] << 8) | buffer[7];
    
    return true;
}

uint8_t SFA30::_crc8(const uint8_t* data, uint8_t len) {
    // Sensirion CRC8 polynomial: x^8 + x^5 + x^4 + 1 (0x31)
    // Initial value: 0xFF
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

bool SFA30::_checkCrc(const uint8_t* data, uint8_t len, uint8_t checksum) {
    return (_crc8(data, len) == checksum);
}