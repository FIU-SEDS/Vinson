#include "BMP390.h"
#include <Arduino.h>
#include <Wire.h>

BMP390::BMP390()
{
}

bool BMP390::begin(uint8_t address)
{
    Wire.begin();
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
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
    uint32_t atmospheric = read_pressure() / 100;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

bool BMP390::set_measurement_mode(uint8_t mode)
{
    return write_register(BMP390_PWR_CTRL, mode);
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
    return write_register(BMP390_CONFIG, fs); // 3-bit filter setting
}

bool BMP390::set_output_data_rate(uint8_t odr)
{
    return write_register(BMP390_ODR, odr); // 5-bit ODR setting
}

// ========== Helper Functions for I2C Communication ==========

bool BMP390::write_register(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t BMP390::read_register(uint8_t reg)
{
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false); // Restart condition
    Wire.requestFrom(BMP390_I2C_ADDRESS, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

void BMP390::read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    Wire.beginTransmission(BMP390_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false); // Restart condition to keep I2C bus control

    Wire.requestFrom(BMP390_I2C_ADDRESS, length);
    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = Wire.available() ? Wire.read() : 0;
    }
}
