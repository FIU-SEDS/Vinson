#pragma once
#ifndef BMP390_I2C_DRIVER_H
#define BMP390_I2C_DRIVER_H

#include <stdint.h>

// UINT8_C is an unsigned 8 bit macro that ensures it stays the same data type regardless of compiler/platform

// ADDRESSES
#define BMP390_I2C_ADDRESS UINT8_C(0x76)

// REGISTERS (p. 30)
#define BMP390_DEVICE_ID UINT8_C(0x00)
#define BMP390_REV_ID UINT8_C(0x01)
#define BMP390_ERROR_REG UINT8_C(0x02)
#define BMP390_STATUS UINT8_C(0x03)
// Pressure Data
#define BMP390_DATA_0 UINT8_C(0x04)
#define BMP390_DATA_1 UINT8_C(0x05)
#define BMP390_DATA_2 UINT8_C(0x06)
// Temperature Data
#define BMP390_DATA_3 UINT8_C(0x07)
#define BMP390_DATA_4 UINT8_C(0x08)
#define BMP390_DATA_5 UINT8_C(0x09)
// Sensor Time
#define BMP390_SENSORTIME_0 UINT8_C(0x0C)
#define BMP390_SENSORTIME_1 UINT8_C(0x0D)
#define BMP390_SENSORTIME_2 UINT8_C(0x0E)

// Sensor Config
#define BMP390_PWR_CTRL UINT8_C(0x1B)
#define BMP390_OSR UINT8_C(0x1C)
#define BMP390_ODR UINT8_C(0x1D)
#define BMP390_IIR_FILTER_CONFIG UINT8_C(0x1F)
#define BMP390_CMD UINT8_C(0x7E)

// Values
#define BMP390_BASE_VALUE UINT8_C(0b00000000)
#define PWR_NORMAL_TP_EN UINT8_C(0b00110011)
#define OVERSAMPLING_P8X_T2X UINT8_C(0b00001011)
#define IIR_FILTER_COEFF_1 UINT8_C(0b00000010)
#define ODR_RATE_50HZ UINT8_C(0x02)
#define SOFT_RESET_VALUE UINT8_C(0xB6)

class BMP390
{

public:
  BMP390(); // Constructor

  bool begin(uint8_t address = BMP390_I2C_ADDRESS);
  uint8_t chip_id();

  int16_t read_temperature();
  int32_t read_pressure();
  int32_t read_altitude(float seaLevel);

  bool set_measurement_mode(uint8_t mode);
  bool set_pressure_temperature_oversampling(uint8_t os);
  bool set_IIR_filter_coeff(uint8_t fs);
  bool set_output_data_rate(uint8_t odr);

private:
  uint8_t i2c_address;

  bool write_register(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
  void read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif