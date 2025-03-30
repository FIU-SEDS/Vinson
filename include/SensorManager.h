#pragma once
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include <Arduino.h>

#define MAIN_IMU_ADDRESS 0x6A

#define RX_PIN 3 // Connect to TX of RYLR998
#define TX_PIN 2 // Connect to RX of RYLR998

#define INTERVAL_APOGEE 1000                   // 1 second (1000ms) to measure if in apogee
#define LIFTOFF_GRAVITY_THRESHOLD 1200         // 1.5g (1.5g = 1500 mg) 1.5g is 1500 mg
#define APOGEE_GRAVITY_THRESHOLD INT16_C(1000) // 1g in mg for near freefall (apogee detection)
#define LANDING_GRAVITY_THRESHOLD 1000         // indicating minimal vertical movement in mg after landing

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define DEBUG 0
#if DEBUG == 1
void logStatus(const char *device, const char *operation, bool success);
#endif

/**
 * @enum RocketState
 * @brief Represents the various states of the rocket during its flight.
 *
 * - INIT_AND_SYSTEMS_CHECK: Initial state where systems are being checked.
 * - IDLE: Rocket is idle detecting for launch.
 * - BOOST: Rocket is in the boost phase.
 * - APOGEE: Rocket has reached the highest point in its trajectory.
 * - DROGUE_DEPLOY: Drogue parachute deployment phase.
 * - MAIN_DEPLOY: Main parachute deployment phase.
 * - LANDED: Rocket has landed.
 */
enum RocketState
{
  INIT_AND_SYSTEMS_CHECK,
  IDLE,
  BOOST,
  APOGEE,
  DROGUE_DEPLOY,
  MAIN_DEPLOY,
  LANDED
};

/**
 * @enum CriticalIndex
 * @brief Indices representing critical sensors in the system.
 *
 * Critical sensors are essential for the safe operation and monitoring of the rocket.
 */
enum CriticalIndex
{
  MAIN_IMU
};

enum Axes
{
  X,
  Y,
  Z
};

/**
 * @enum NonCriticalIndex
 * @brief Indices representing non-critical sensors in the system.
 *
 * Non-critical sensors provide additional data but are not essential for the rocket's primary functions.
 */
enum NonCriticalIndex
{
  MAGNETOMETER
};

// Initializes LoRa radio
void InitializeLoRa();

// Log status
void logStatus(const char *device, const char *operation, bool success);

// Initializes and verifies all critical and non-critical sensors.
bool InitializeAndCheckSensors();

// Checks the I2C connection for a device based on its address.
bool isDeviceConnected(uint8_t address);

// Determines if rocket is in BOOST phase as it ensures that either acceleration or altitude is increasing
bool CheckLiftoffConditions();

// Determines if the rocket reached APOGEE phase
bool CheckApogeeConditions();

// Determines if the rocket has reached apogee to deploy drogue parachutes
bool CheckDrogueDeployment();

// Deploys drogue parachute one second after apogee is detected
void DeployDrogueParachute();

// Determines if the rocket has landed
bool CheckLandingConditions();

// Begins data transmission of all the sensors and store it inside the SD Card
void StartData(RocketState current_state);

// If rocket has landed then stops saving data to SD card and shuts down
void DumpData();

#endif