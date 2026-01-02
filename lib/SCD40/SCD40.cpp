#include "SCD40.h"
#include <math.h>

// Command definitions
const uint16_t SCD40::CMD_START_PERIODIC_MEAS = 0x21B1;
const uint16_t SCD40::CMD_STOP_PERIODIC_MEAS = 0x3F86;
const uint16_t SCD40::CMD_READ_MEAS = 0xEC05;
const uint16_t SCD40::CMD_DATA_READY = 0xE4B8;
const uint16_t SCD40::CMD_SET_TEMP_OFFSET = 0x241D;
const uint16_t SCD40::CMD_GET_TEMP_OFFSET = 0x2318;
const uint16_t SCD40::CMD_SET_ALTITUDE = 0x2427;
const uint16_t SCD40::CMD_GET_ALTITUDE = 0x2322;
const uint16_t SCD40::CMD_SET_AMBIENT_PRESSURE = 0xE000;
const uint16_t SCD40::CMD_FORCED_RECAL = 0x362F;
const uint16_t SCD40::CMD_SET_AUTO_CALIB = 0x2416;
const uint16_t SCD40::CMD_GET_AUTO_CALIB = 0x2313;
const uint16_t SCD40::CMD_GET_SERIAL = 0x3682;
const uint16_t SCD40::CMD_GET_FIRMWARE = 0x202F;
const uint16_t SCD40::CMD_PERFORM_FACTORY_RESET = 0x3632;
const uint16_t SCD40::CMD_REINIT = 0x3646;
const uint16_t SCD40::CMD_SOFT_RESET = 0x0006;

SCD40::SCD40(TwoWire& wire, uint8_t address) 
    : wire(wire), address(address), measurementActive(false), errorCount(0), lastReadTime(0) {}

bool SCD40::begin() {
    wire.begin();
    delay(100);
    
    // Check if sensor is connected
    if (!isConnected()) {
        return false;
    }
    
    // Perform soft reset
    if (!writeCommand(CMD_SOFT_RESET)) {
        errorCount++;
        return false;
    }
    
    delay(2000); // Wait for sensor to initialize after reset
    
    // Reinitialize
    if (!writeCommand(CMD_REINIT)) {
        errorCount++;
        return false;
    }
    
    delay(100);
    
    return true;
}

bool SCD40::startMeasurement() {
    if (!writeCommand(CMD_START_PERIODIC_MEAS)) {
        errorCount++;
        return false;
    }
    
    measurementActive = true;
    delay(100);
    return true;
}

bool SCD40::stopMeasurement() {
    if (!writeCommand(CMD_STOP_PERIODIC_MEAS)) {
        errorCount++;
        return false;
    }
    
    measurementActive = false;
    delay(100);
    return true;
}

bool SCD40::readData(Data& data) {
    if (!measurementActive) {
        return false;
    }
    
    // Check if data is ready
    if (!isDataReady()) {
        data.valid = false;
        data.dataReady = false;
        return false;
    }
    
    uint8_t buffer[9]; // 3 measurements * (2 bytes + 1 CRC)
    
    if (!writeCommand(CMD_READ_MEAS)) {
        errorCount++;
        return false;
    }
    
    delay(1); // Small delay
    
    if (!readResponse(buffer, 9)) {
        errorCount++;
        return false;
    }
    
    // Check CRCs for each measurement
    for (int i = 0; i < 3; i++) {
        if (!checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            errorCount++;
            return false;
        }
    }
    
    uint16_t co2_raw = (buffer[0] << 8) | buffer[1];
    uint16_t temp_raw = (buffer[3] << 8) | buffer[4];
    uint16_t hum_raw = (buffer[6] << 8) | buffer[7];
    
    data.co2 = co2_raw; // Direct conversion for SCD40
    data.temperature = convertTemperature(temp_raw);
    data.humidity = convertHumidity(hum_raw);
    data.valid = true;
    data.dataReady = true;
    data.timestamp = millis();
    lastReadTime = millis();
    
    return true;
}

bool SCD40::isDataReady() {
    uint8_t buffer[3];
    
    if (!writeCommand(CMD_DATA_READY)) {
        errorCount++;
        return false;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 3)) {
        errorCount++;
        return false;
    }
    
    if (!checkCRC(buffer, 2, buffer[2])) {
        errorCount++;
        return false;
    }
    
    uint16_t ready = (buffer[0] << 8) | buffer[1];
    return (ready & 0x07FF) != 0; // Check if data is ready
}

bool SCD40::setTemperatureOffset(float offset) {
    if (offset < -10.0f || offset > 10.0f) {
        return false;
    }
    
    uint16_t offset_raw = (uint16_t)(offset * 100.0f);
    if (!writeCommandWithData(CMD_SET_TEMP_OFFSET, &offset_raw, 1)) {
        errorCount++;
        return false;
    }
    
    delay(100);
    return true;
}

float SCD40::getTemperatureOffset() {
    uint8_t buffer[3];
    
    if (!writeCommand(CMD_GET_TEMP_OFFSET)) {
        errorCount++;
        return 0.0f;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 3)) {
        errorCount++;
        return 0.0f;
    }
    
    if (!checkCRC(buffer, 2, buffer[2])) {
        errorCount++;
        return 0.0f;
    }
    
    uint16_t offset_raw = (buffer[0] << 8) | buffer[1];
    return offset_raw / 100.0f;
}

bool SCD40::setAutomaticSelfCalibration(bool enabled) {
    uint16_t value = enabled ? 1 : 0;
    if (!writeCommandWithData(CMD_SET_AUTO_CALIB, &value, 1)) {
        errorCount++;
        return false;
    }
    
    delay(100);
    return true;
}

bool SCD40::getAutomaticSelfCalibration() {
    uint8_t buffer[3];
    
    if (!writeCommand(CMD_GET_AUTO_CALIB)) {
        errorCount++;
        return false;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 3)) {
        errorCount++;
        return false;
    }
    
    if (!checkCRC(buffer, 2, buffer[2])) {
        errorCount++;
        return false;
    }
    
    uint16_t asc_status = (buffer[0] << 8) | buffer[1];
    return (asc_status == 1);
}

bool SCD40::setSensorAltitude(uint16_t altitude) {
    if (!writeCommandWithData(CMD_SET_ALTITUDE, &altitude, 1)) {
        errorCount++;
        return false;
    }
    
    delay(100);
    return true;
}

uint16_t SCD40::getSensorAltitude() {
    uint8_t buffer[3];
    
    if (!writeCommand(CMD_GET_ALTITUDE)) {
        errorCount++;
        return 0;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 3)) {
        errorCount++;
        return 0;
    }
    
    if (!checkCRC(buffer, 2, buffer[2])) {
        errorCount++;
        return 0;
    }
    
    return (buffer[0] << 8) | buffer[1];
}

bool SCD40::setAmbientPressure(uint16_t pressure) {
    if (pressure < 70000 || pressure > 120000) {
        return false; // Pressure in Pa
    }
    
    if (!writeCommandWithData(CMD_SET_AMBIENT_PRESSURE, &pressure, 1)) {
        errorCount++;
        return false;
    }
    
    delay(100);
    return true;
}

bool SCD40::performForcedRecalibration(uint16_t target_co2) {
    if (target_co2 < 400 || target_co2 > 2000) {
        return false;
    }
    
    if (!writeCommandWithData(CMD_FORCED_RECAL, &target_co2, 1)) {
        errorCount++;
        return false;
    }
    
    delay(400); // Wait for calibration to complete
    return true;
}

bool SCD40::performFactoryReset() {
    if (!writeCommand(CMD_PERFORM_FACTORY_RESET)) {
        errorCount++;
        return false;
    }
    
    delay(1200); // Factory reset takes up to 1.2 seconds
    return true;
}

uint64_t SCD40::getSerialNumber() {
    uint8_t buffer[9]; // 3 words (6 bytes) + 3 CRC bytes
    
    if (!writeCommand(CMD_GET_SERIAL)) {
        errorCount++;
        return 0;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 9)) {
        errorCount++;
        return 0;
    }
    
    // Check CRCs
    for (int i = 0; i < 3; i++) {
        if (!checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            errorCount++;
            return 0;
        }
    }
    
    uint64_t serial = 0;
    serial = (serial << 16) | ((buffer[0] << 8) | buffer[1]);
    serial = (serial << 16) | ((buffer[3] << 8) | buffer[4]);
    serial = (serial << 16) | ((buffer[6] << 8) | buffer[7]);
    
    return serial;
}

String SCD40::getSerialNumberString() {
    uint64_t serial = getSerialNumber();
    if (serial == 0) {
        return "Unknown";
    }
    
    char buffer[13]; // 12 hex digits + null terminator
    sprintf(buffer, "%08lX", (uint32_t)(serial >> 32));
    sprintf(buffer + 8, "%08lX", (uint32_t)(serial & 0xFFFFFFFF));
    return String(buffer);
}

uint16_t SCD40::getFirmwareVersion() {
    uint8_t buffer[3];
    
    if (!writeCommand(CMD_GET_FIRMWARE)) {
        errorCount++;
        return 0;
    }
    
    delay(1);
    
    if (!readResponse(buffer, 3)) {
        errorCount++;
        return 0;
    }
    
    if (!checkCRC(buffer, 2, buffer[2])) {
        errorCount++;
        return 0;
    }
    
    return (buffer[0] << 8) | buffer[1];
}

bool SCD40::isConnected() {
    wire.beginTransmission(address);
    return (wire.endTransmission() == 0);
}

bool SCD40::isMeasuring() {
    return measurementActive;
}

String SCD40::getCO2Quality(uint16_t co2) {
    if (co2 < 450) return "Outdoor Fresh";
    else if (co2 < 800) return "Excellent";
    else if (co2 < 1000) return "Good";
    else if (co2 < 1200) return "Moderate";
    else if (co2 < 1500) return "Poor";
    else return "Unhealthy";
}

String SCD40::getCO2Recommendation(uint16_t co2) {
    if (co2 < 800) return "Maintain ventilation";
    else if (co2 < 1000) return "Increase ventilation";
    else if (co2 < 1200) return "Open windows";
    else return "VENTILATE NOW";
}

float SCD40::calculateDewPoint(float temperature, float humidity) {
    // Magnus formula
    if (humidity > 100) humidity = 100;
    if (humidity < 0) humidity = 0;
    
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

bool SCD40::writeCommand(uint16_t command) {
    wire.beginTransmission(address);
    wire.write(command >> 8);
    wire.write(command & 0xFF);
    return (wire.endTransmission() == 0);
}

bool SCD40::writeCommandWithData(uint16_t command, const uint16_t* data, uint8_t data_words) {
    uint8_t buffer[2 + data_words * 3]; // Command + data words with CRC
    
    buffer[0] = command >> 8;
    buffer[1] = command & 0xFF;
    
    for (uint8_t i = 0; i < data_words; i++) {
        buffer[2 + i*3] = data[i] >> 8;
        buffer[3 + i*3] = data[i] & 0xFF;
        buffer[4 + i*3] = crc8(&buffer[2 + i*3], 2);
    }
    
    wire.beginTransmission(address);
    wire.write(buffer, sizeof(buffer));
    return (wire.endTransmission() == 0);
}

bool SCD40::readResponse(uint8_t* buffer, uint8_t length, uint8_t delay_ms) {
    delay(delay_ms);
    
    wire.requestFrom(address, length);
    
    if (wire.available() < length) {
        return false;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = wire.read();
    }
    
    return true;
}

bool SCD40::readResponseWithCRC(uint16_t* data, uint8_t data_words) {
    uint8_t buffer[data_words * 3];
    
    if (!readResponse(buffer, data_words * 3)) {
        return false;
    }
    
    for (uint8_t i = 0; i < data_words; i++) {
        if (!checkCRC(&buffer[i*3], 2, buffer[i*3 + 2])) {
            return false;
        }
        data[i] = (buffer[i*3] << 8) | buffer[i*3 + 1];
    }
    
    return true;
}

uint8_t SCD40::crc8(uint8_t data[], uint8_t len) {
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

bool SCD40::checkCRC(uint8_t data[], uint8_t len, uint8_t checksum) {
    return (crc8(data, len) == checksum);
}

float SCD40::convertTemperature(uint16_t raw) {
    return -45.0f + 175.0f * (float)raw / 65535.0f;
}

float SCD40::convertHumidity(uint16_t raw) {
    return 100.0f * (float)raw / 65535.0f;
}