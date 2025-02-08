#include "BMP390.h"
#include <Wire.h>

BMP390::BMP390()
{
    i2c_address = BMP390_I2C_ADDRESS;
}

bool BMP390::begin(uint8_t address)
{
    Wire.begin();
    i2c_address = address;
    Wire.beginTransmission(i2c_address);
    return (Wire.endTransmission() == 0); // Return true if BMP390 responds
}

uint8_t BMP390::chip_id()
{
    return read_register(BMP390_DEVICE_ID);
}

int16_t BMP390::read_temperature()
{
    uint8_t temp_raw[3];
    read_bytes(BMP390_DATA_3, temp_raw, 3);
    int32_t temp = ((int32_t)temp_raw[2] << 16) | ((int32_t)temp_raw[1] << 8) | temp_raw[0];
    return temp / 100; // Temperature in °C
}

int32_t BMP390::read_pressure()
{
    uint8_t press_raw[3];
    read_bytes(BMP390_DATA_0, press_raw, 3);
    int32_t pressure = ((int32_t)press_raw[2] << 16) | ((int32_t)press_raw[1] << 8) | press_raw[0];
    return pressure / 256; // Pressure in Pascals
}

int32_t BMP390::read_altitude(float seaLevel)
{
    // Read pressure (in hPa) as an integer
    int32_t pressure = read_pressure();

    // Convert sea level pressure to hPa (as integer), assuming seaLevel is in hPa
    int32_t seaLevelPressure = static_cast<int32_t>(seaLevel * 100); // Convert seaLevel to hPa (e.g., 1013.25 hPa = 101325)

    // Calculate the ratio (pressure/seaLevel) as integer
    int32_t ratio = (pressure * 1000) / seaLevelPressure; // Multiply by 1000 for precision

    // Approximate power of ratio^5 using integer math (no floating point)
    ratio = ratio * ratio * ratio * ratio * ratio; // ratio^5

    // Calculate altitude in meters using integer math (avoiding floating point)
    int32_t altitudeInMeters = 44330 * (1000 - ratio) / 1000; // Integer-only calculation

    // Convert altitude to feet (1 meter = 3.28084 feet) using integer math
    int32_t altitudeInFeet = (altitudeInMeters * 328084) / 100000; // Integer division to avoid float

    return altitudeInFeet; // Return altitude in feet
}

bool BMP390::set_temperature_oversampling(uint8_t os)
{
    return write_register(BMP390_OSR, (os & 0x07)); // 3-bit oversampling setting
}

bool BMP390::set_pressure_oversampling(uint8_t os)
{
    return write_register(BMP390_OSR, (os & 0x07) << 3); // Shift bits for pressure OSR
}

bool BMP390::set_IIR_filter_coeff(uint8_t fs)
{
    return write_register(BMP390_CONFIG, (fs & 0x07)); // 3-bit filter setting
}

bool BMP390::set_output_data_rate(uint8_t odr)
{
    return write_register(BMP390_ODR, (odr & 0x1F)); // 5-bit ODR setting
}

// ========== Helper Functions for I2C Communication ==========

bool BMP390::write_register(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t BMP390::read_register(uint8_t reg)
{
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.endTransmission(false); // Restart condition
    Wire.requestFrom(i2c_address, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

void BMP390::read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.endTransmission(false); // Restart condition to keep I2C bus control
    Wire.requestFrom(i2c_address, length);
    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = Wire.available() ? Wire.read() : 0;
    }
}
