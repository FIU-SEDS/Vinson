#pragma once
#ifndef BMP390_I2C_DRIVER_H
#define BMP390_I2C_DRIVER_H

#include <stdint.h>

// ADDRESSES
#define BMP390_I2C_ADDRESS 0x76

// REGISTERS (p. 30)
#define BMP390_DEVICE_ID 0x00
#define BMP390_REV_ID 0x01
#define BMP390_ERROR_REG 0x02
#define BMP390_STATUS 0x03
// Pressure Data
#define BMP390_DATA_0 0x04
#define BMP390_DATA_1 0x05
#define BMP390_DATA_2 0x06
// Temperature Data
#define BMP390_DATA_3 0x07
#define BMP390_DATA_4 0x08
#define BMP390_DATA_5 0x09
// Sensor Time
#define BMP390_SENSORTIME_0 0x0C
#define BMP390_SENSORTIME_1 0x0D
#define BMP390_SENSORTIME_2 0x0E

#define BMP390_PWR_CTRL 0x1B

// Sensor Config
#define BMP390_OSR 0x1C
#define BMP390_ODR 0x1D
#define BMP390_CONFIG 0x1F
#define BMP390_CMD 0x7E
class BMP390
{

public:
  BMP390(); // Constructor

  bool begin(uint8_t address = BMP390_I2C_ADDRESS);
  uint8_t chip_id();

  int16_t read_temperature();
  int32_t read_pressure();
  int32_t read_altitude(float seaLevel);

  bool set_temperature_oversampling(uint8_t os);
  bool set_pressure_oversampling(uint8_t os);
  bool set_IIR_filter_coeff(uint8_t fs);
  bool set_output_data_rate(uint8_t odr); // Fixed Typo

private:
  uint8_t i2c_address;

  bool write_register(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
  void read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif