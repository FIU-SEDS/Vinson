#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include <Arduino.h>

#define MAGNETOMETER_ADDRESS 0x30
#define MAIN_IMU_ADDRESS 0x6A
#define BAROMETER_ADDRESS 0x76

enum RocketState {INIT_AND_SYSTEMS_CHECK, IDLE, BOOST, APOGEE, DROGUE, MAIN, LANDED};

enum CriticalIndex
{
  MAIN_IMU,
  BAROMETER
}; // Indices for critical sensors
enum NonCriticalIndex
{
  MAGNETOMETER
}; // Indices for non-critical sensors

// Function prototypes
bool InitializeAndCheckSensors();

// MMC5983 Magnetometer
bool PowerMagnetometer();
bool MagnetometerVerifyTemperature();
bool MagnetometerVerifyConnection();

// ASM330LHH Main IMU
bool PowerMainIMU();
bool MainIMUVerifyTemperature();
bool MainIMUVerifyConnection();

// BMP390 Barometer
bool PowerBarometer();
bool BarometerVerifyTemperature();
bool BarometerVerifyConnection();

bool isDeviceConnected(uint8_t address);

#endif



