#pragma once
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include <Arduino.h>

#define MAIN_IMU_ADDRESS 0x6A
#define BAROMETER_ADDRESS 0x76

#define SEA_LEVEL_PRESSURE (1012.32)             // provide sea-level pressure (in hPa)
#define FEET_PER_METER 3.28084                 // From meters to feet conversion value
#define MAIN_DEPLOYMENT_ALTITUDE 1000.0        // Main parachute Deployment altitude in feet
#define LIFTOFF_ALTITUDE_THRESHOLD 2          // Liftoff level in feet
#define LIFTOFF_GRAVITY_THRESHOLD 1500         // 1.5g (1.5g = 1500 mg) 1.5g is 1500 mg which is the unit the IMU measures in
#define INTERVAL_APOGEE 1000                   // 1 second (1000 ms) interval to measure altitude
#define APOGEE_GRAVITY_THRESHOLD INT16_C(1000) // 1g in mg for near freefall (apogee detection)
#define APOGEE_ALTITUDE_THRESHOLD 10           // 10 ft minimum descent for descent detection
#define LANDING_ALTITUDE 30                    // Ground level altitude in feet (can adjust for launch site)
#define LANDING_GRAVITY_THRESHOLD 500          // indicating minimal vertical movement in mg after landing

#define DEBUG 0
#if DEBUG == 1
Serial.println(x); // Ensure 'x' is defined or replace it with the proper debug message.
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
  MAIN_IMU,
  BAROMETER
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
// enum NonCriticalIndex
// {
//   MAGNETOMETER,
//   HTURHT // HTU Relative Humidity and Temperature sensor
// };

// Initializes and verifies all critical and non-critical sensors.
bool InitializeAndCheckSensors();

// Checks the I2C connection for a device based on its address.
bool isDeviceConnected(uint8_t address);

// Determines if rocket is in BOOST phase as it ensures that either acceleration or altitude is increasing
bool CheckLiftoffConditions();

// Determines if the rocket reached APOGEE phase
bool CheckApogeeConditions();

// Determines if the rocket has reached altitude to deploy drogue parachutes
bool CheckDrogueDeployment();

// Deploys drogue parachute five seconds after Apogee is detected
void DeployDrogueParachute();

// Determines if the rocket has reached certain altitude to deploy main parachutes
bool CheckMainDeploymentConditions();

// Deploys main parachute after verification that certain altitude is reached
void DeployMainParachute();

// Determines if the rocket has landed
bool CheckLandingConditions();

// Begins data transmission of all the sensors and store it inside the SD Card
void StartData();

// If rocket has landed then stops saving data to SD card and shuts down
void DumpData();

// Powers up the Barometer sensor.
bool PowerBarometer();

#endif