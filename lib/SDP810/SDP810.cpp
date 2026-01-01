#include "SDP810.h"

SDP810::SDP810(TwoWire& wire, uint8_t address) 
    : _wire(wire), _address(address) {}

bool SDP810::begin() {
    _wire.begin();
    delay(100);
    
    // Check if sensor is connected
    _wire.beginTransmission(_address);
    return (_wire.endTransmission() == 0);
}

bool SDP810::startContinuousMeasurement(MeasurementMode mode) {
    _writeCommand(mode);
    delay(20); // Wait for sensor to start
    return true;
}

bool SDP810::readMeasurement(Data& data) {
    uint16_t raw_pressure, raw_temperature;
    uint8_t status;
    
    if (!_readRawData(raw_pressure, raw_temperature, status)) {
        return false;
    }
    
    data.raw_pressure = raw_pressure;
    data.raw_temperature = raw_temperature;
    data.status = status;
    data.differential_pressure = _convertPressure(raw_pressure);
    data.temperature = _convertTemperature(raw_temperature);
    data.air_flow = calculateAirFlow(data.differential_pressure);
    data.air_velocity = calculateAirVelocity(data.differential_pressure);
    
    return true;
}

bool SDP810::triggerMeasurement(Data& data) {
    // Send trigger command
    _writeCommand(TRIGGERED_MEASUREMENT);
    delay(20); // Wait for measurement
    
    return readMeasurement(data);
}

bool SDP810::softReset() {
    _writeCommand(0x0006); // Soft reset command
    delay(100); // Wait for reset
    return true;
}

float SDP810::calculateAirFlow(float pressure, float diameter) {
    // Using Bernoulli's equation for incompressible flow
    // Q = A * √(2ΔP/ρ)
    // Where: A = cross-sectional area, ρ = air density (1.225 kg/m³ at sea level)
    
    if (pressure < 0) pressure = -pressure; // Use absolute value
    
    float radius = diameter / 2.0;
    float area = PI * radius * radius; // Cross-sectional area
    float density = 1.225; // kg/m³ at 15°C, sea level
    
    // Convert Pa to flow rate (m³/s)
    float flow = area * sqrt(2.0 * pressure / density);
    
    return flow;
}

float SDP810::calculateAirVelocity(float pressure, float diameter) {
    // v = √(2ΔP/ρ)
    float density = 1.225; // kg/m³
    
    if (pressure < 0) pressure = -pressure; // Use absolute value
    
    return sqrt(2.0 * pressure / density);
}

bool SDP810::isConnected() {
    _wire.beginTransmission(_address);
    return (_wire.endTransmission() == 0);
}

bool SDP810::_readRawData(uint16_t& pressure, uint16_t& temperature, uint8_t& status) {
    uint8_t buffer[9];
    
    // Request 9 bytes (3 bytes per value + checksum)
    _wire.requestFrom(_address, (uint8_t)9);
    
    if (_wire.available() < 9) {
        return false;
    }
    
    for (int i = 0; i < 9; i++) {
        buffer[i] = _wire.read();
    }
    
    // Verify checksum (CRC8)
    // For simplicity, we'll skip checksum verification in this basic version
    // You can add CRC8 calculation if needed
    
    pressure = (buffer[0] << 8) | buffer[1];
    temperature = (buffer[3] << 8) | buffer[4];
    status = buffer[6];
    
    return true;
}

float SDP810::_convertPressure(uint16_t raw) {
    // Convert raw value to Pascal
    // SDP810-500Pa range: -500 to +500 Pa
    int16_t signed_raw = (int16_t)raw;
    return (signed_raw / 60.0); // Scale factor from datasheet
}

float SDP810::_convertTemperature(uint16_t raw) {
    // Convert raw value to Celsius
    return (raw / 200.0);
}

void SDP810::_writeCommand(uint16_t command) {
    _wire.beginTransmission(_address);
    _wire.write(command >> 8);    // MSB
    _wire.write(command & 0xFF);  // LSB
    _wire.endTransmission();
}

bool SDP810::_readResponse(uint8_t* buffer, uint8_t length) {
    _wire.requestFrom(_address, length);
    
    if (_wire.available() < length) {
        return false;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = _wire.read();
    }
    
    return true;
}