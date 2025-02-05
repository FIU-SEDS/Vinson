#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include <Arduino.h>

#define MAGNETOMETER_ADDRESS 0x30
#define MAIN_IMU_ADDRESS 0x6A
#define BAROMETER_ADDRESS 0x76
// #define HTU_ADDRESS 0x40

#define SEA_LEVEL_PRESSURE 1013.25      // provide sea-level pressure (in hPa)
#define MAIN_DEPLOYMENT_ALTITUDE 1000.0 // Main parachute Deployment altitude in feet
#define LANDING_ALTITUDE 50             // Ground level altitude in feet (can adjust for launch site)
#define ACCELERATION_THRESHOLD 10       // Acceleration threshold for "still" in m/s² (near zero)

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

/**
 * @enum NonCriticalIndex
 * @brief Indices representing non-critical sensors in the system.
 *
 * Non-critical sensors provide additional data but are not essential for the rocket's primary functions.
 */
enum NonCriticalIndex
{
  MAGNETOMETER,
  HTURHT // HTU Relative Humidity and Temperature sensor
};

/////////////////////////////////////////// Function prototypes ///////////////////////////////////////////

// Initializes and verifies all critical and non-critical sensors.
bool InitializeAndCheckSensors();

// Checks the I2C connection for a device based on its address.
bool isDeviceConnected(uint8_t address);

// Begins data transmission of all the sensors and store it inside the SD Card
void StartData();

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

// If rocket has landed then stops saving data to SD card and shuts down
void DumpData();

/////////////////////////////////////////// MMC5983 Magnetometer ///////////////////////////////////////////

// Powers up the Magnetometer sensor.
bool PowerMagnetometer();

// Verifies the temperature of the Magnetometer sensor.
bool MagnetometerVerifyTemperature();

// Verifies the I2C connection of the Magnetometer sensor.
bool MagnetometerVerifyConnection();

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////// ASM330LHH Main IMU ///////////////////////////////////////////

// Powers up the Main IMU sensor.
bool PowerMainIMU();

// Verifies the temperature of the Main IMU sensor.
bool MainIMUVerifyTemperature();

// Verifies the I2C connection of the Main IMU sensor.
bool MainIMUVerifyConnection();

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////// BMP390 Barometer ///////////////////////////////////////////

// Powers up the Barometer sensor.
bool PowerBarometer();

// Verifies the temperature of the Barometer sensor.
bool BarometerVerifyTemperature();

// Verifies the I2C connection of the Barometer sensor.
bool BarometerVerifyConnection();

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////// HTU20DF Humidity & Temperature Sensor ///////////////////////////////////////////

// Powers up the HTU sensor
bool PowerHTU();

// Verifies the I2C connection of the HTU sensor.
bool HTUVerifyConnection();

#endif