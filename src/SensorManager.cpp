#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> // Magnetometer Library
#include <ASM330LHHSensor.h>                    // Main IMU Library
#include <Adafruit_BMP3XX.h>                    // Barometer Library

bool criticalSensors[2];
bool nonCriticalSensors[1];

// Sensor objects
SFE_MMC5983MA magnetometer;
ASM330LHHSensor mainIMU(&Wire, ASM330LHH_I2C_ADD_L);
Adafruit_BMP3XX barometer;

// Initial readings for reference
float initialAltitude = 0;
float initialAcceleration = 0;
float initialMagnitude = 0;
float lastAltitude = 0;
float lastAccel = 0;
int32_t mainIMUInitAccelAxes[3] = {};
// Goes from x, y, z for initital acceleration
int32_t mainIMUCurrAccelAxes[3] = {};
// Goes from x, y, z for current acceleration

/*
 * @brief Prints out a log message for the state of a given parameter.
 *
 * This function logs the device's operation status, indicating whether it succeeded or failed.
 *
 * @param device Name of the device (e.g., "Magnetometer").
 * @param operation Description of the operation being performed (e.g., "Power Up").
 * @param success Boolean indicating the success (true) or failure (false) of the operation.
 * @return None. The function prints a statement to the serial log.
 */
void logStatus(const char *device, const char *operation, bool success)
{
  if (success)
  {
    Serial.print("[INFO] ");
    Serial.print(device);
    Serial.print(": ");
    Serial.print(operation);
    Serial.println(" succeeded.");
  }
  else
  {
    Serial.print("[ERROR] ");
    Serial.print(device);
    Serial.print(": ");
    Serial.print(operation);
    Serial.println(" failed.");
  }
}

/*
 * @brief First check of the I2C connection for a device.
 *
 * This function uses the Wire library to initiate a transmission to the specified I2C address
 * and verifies if the device responds, indicating a successful connection.
 *
 * @param address I2C address of the sensor to check.
 * @return Boolean value: true if the device is connected and responds, false otherwise.
 */
bool isDeviceConnected(uint8_t address)
{
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0); // Returns true if device responds
}

/*
 * @brief Powers up the Magnetometer sensor.
 *
 * This function checks the I2C connection to the Magnetometer. If connected, it initializes the
 * magnetometer and performs a soft reset. Logs the status of the power-up process.
 *
 * @return Boolean value: true if the Magnetometer is successfully powered up, false otherwise.
 */
bool PowerMagnetometer()
{
  if (!isDeviceConnected(MAGNETOMETER_ADDRESS))
    return logStatus("Magnetometer", "I2C Start Device Check (Power Up Function)", false), false;

  if (magnetometer.begin())
  {
    magnetometer.softReset();
    logStatus("Magnetometer", "Power Up", true);
    return true;
  }
  return logStatus("Magnetometer", "Power Up", false), false;
}

/*
 * @brief Verifies the temperature of the Main IMU sensor.
 *
 * This function checks if the Main IMU's temperature is within the acceptable range.
 * Logs the result of the temperature verification.
 *
 * @return Boolean value: true if the temperature check passes, false otherwise.
 */
bool MagnetometerVerifyTemperature()
{
  if (magnetometer.verifyTemperature())
    return logStatus("Magnetometer", "Temperature Check", true), true;

  return logStatus("Magnetometer", "Temperature Check", false), false;
}

/*
 * @brief Verifies the I2C connection of the Magnetometer sensor.
 *
 * This function checks if the Magnetometer is properly connected via I2C by verifying its address.
 * Logs the result of the connection verification.
 *
 * @return Boolean value: true if the I2C connection is verified, false otherwise.
 */
bool MagnetometerVerifyConnection()
{
  if (magnetometer.verifyConnection(MAGNETOMETER_ADDRESS))
    return logStatus("Magnetometer", "I2C Connection", true), true;

  return logStatus("Magnetometer", "I2C Connection", false), false;
}

/*
 * @brief Powers up the Main IMU sensor.
 *
 * This function checks the I2C connection to the Main IMU. If connected, it initializes the IMU,
 * enables the accelerometer and gyroscope, and logs the status of the power-up process.
 *
 * @return Boolean value: true if the Main IMU is successfully powered up, false otherwise.
 */
bool PowerMainIMU()
{
  if (!isDeviceConnected(MAIN_IMU_ADDRESS))
    return logStatus("Main IMU", "I2C Start Device Check (Power Up Function)", false), false;

  if (mainIMU.begin() == ASM330LHH_OK)
  {
    mainIMU.Enable_X();
    mainIMU.Enable_G();
    mainIMU.Get_X_Axes(mainIMUInitAccelAxes);
    initialMagnitude = sqrt(pow(mainIMUInitAccelAxes[0], 2) + pow(mainIMUInitAccelAxes[1], 2) + pow(mainIMUInitAccelAxes[2], 2));
    logStatus("Main IMU", "Power Up", true);
    return true;
  }
  return logStatus("Main IMU", "Power Up", false), false;
}

/*
 * @brief Verifies the temperature of the Main IMU sensor.
 *
 * This function checks if the Main IMU's temperature is within the acceptable range.
 * Logs the result of the temperature verification.
 *
 * @return Boolean value: true if the temperature check passes, false otherwise.
 */
bool MainIMUVerifyTemperature()
{
  if (mainIMU.verifyTemperature() == ASM330LHH_OK)
    return logStatus("Main IMU", "Temperature Check", true), true;

  return logStatus("Main IMU", "Temperature Check", false), false;
}

/*
 * @brief Verifies the I2C connection of the Main IMU sensor.
 *
 * This function checks if the Main IMU is properly connected via I2C by verifying its address.
 * Logs the result of the connection verification.
 *
 * @return Boolean value: true if the I2C connection is verified, false otherwise.
 */
bool MainIMUVerifyConnection()
{
  if (mainIMU.verifyConnection(MAIN_IMU_ADDRESS) == ASM330LHH_OK)
    return logStatus("Main IMU", "I2C Connection", true), true;

  return logStatus("Main IMU", "I2C Connection", false), false;
}

/*
 * @brief Powers up the Barometer sensor.
 *
 * This function checks the I2C connection to the Barometer. If connected, it initializes the
 * barometer, sets oversampling rates, filter coefficients, and output data rate. Logs the status
 * of the power-up process.
 *
 * @return Boolean value: true if the Barometer is successfully powered up, false otherwise.
 */
bool PowerBarometer()
{
  if (!isDeviceConnected(BAROMETER_ADDRESS))
    return logStatus("Barometer", "I2C Start Device Check (Power Up Function)", false), false;

  if (barometer.begin_I2C(BAROMETER_ADDRESS))
  {
    barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    barometer.setOutputDataRate(BMP3_ODR_50_HZ);
    initialAltitude = barometer.readAltitude(1013.25); // using initialAltitude from .h file to set the sea-level altitude as baseline
    logStatus("Barometer", "Power Up", true);
    return true;
  }
  return logStatus("Barometer", "Power Up", false), false;
}

/*
 * @brief Verifies the temperature of the Barometer sensor.
 *
 * This function checks if the Barometer's temperature is within the acceptable range.
 * Logs the result of the temperature verification.
 *
 * @return Boolean value: true if the temperature check passes, false otherwise.
 */
bool BarometerVerifyTemperature()
{
  if (barometer.verifyTemperature())
    return logStatus("Barometer", "Temperature Check", true), true;

  return logStatus("Barometer", "Temperature Check", false), false;
}

/*
 * @brief Verifies the I2C connection of the Barometer sensor.
 *
 * This function checks if the Barometer is properly connected via I2C by verifying its address.
 * Logs the result of the connection verification.
 *
 * @return Boolean value: true if the I2C connection is verified, false otherwise.
 */
bool BarometerVerifyConnection()
{
  if (barometer.verifyConnection(BAROMETER_ADDRESS))
    return logStatus("Barometer", "I2C Connection", true), true;

  return logStatus("Barometer", "I2C Connection", false), false;
}

/*
 * @brief Initializes and checks all sensors.
 *
 * This function powers up and verifies the temperature and I2C connections for all critical
 * and non-critical sensors. It logs the status of each sensor and determines if all critical
 * sensors are operational.
 *
 * @return Boolean value: true if all critical sensors are successfully initialized and verified,
 *         false if any critical sensor fails.
 */
bool InitializeAndCheckSensors()
{

  criticalSensors[MAIN_IMU] = PowerMainIMU() && MainIMUVerifyTemperature() && MainIMUVerifyConnection();
  criticalSensors[BAROMETER] = PowerBarometer() && BarometerVerifyTemperature() && BarometerVerifyConnection();

  nonCriticalSensors[MAGNETOMETER] = PowerMagnetometer() && MagnetometerVerifyTemperature() && MagnetometerVerifyConnection();

  // Log the status of critical sensors
  Serial.println("[INFO] Critical Sensor Status:");
  Serial.print("  Main IMU: ");
  Serial.println(criticalSensors[MAIN_IMU] ? "SUCCESS" : "FAILURE");
  Serial.print("  Barometer: ");
  Serial.println(criticalSensors[BAROMETER] ? "SUCCESS" : "FAILURE");

  // Log the status of non-critical sensors
  Serial.println("[INFO] Non-Critical Sensor Status:");
  Serial.print("  Magnetometer: ");
  Serial.println(nonCriticalSensors[MAGNETOMETER] ? "SUCCESS" : "FAILURE");

  // Check if all critical sensors passed
  for (int i = 0; i < 2; i++)
  {
    if (!criticalSensors[i])
    {
      return false; // Halt if any critical sensor fails
    }
  }

  return true; // All critical sensors are operational
}

/*
 * @brief Checks if rocket is in BOOST phase.
 *
 * Ensures that either acceleration or altitude is increasing to verify successful rocket launch
 *
 * @return Boolean value: true if data is successfully verified transitions to BOOST phase, false if conditions arent met.
 *
 */
bool CheckLiftoffConditions()
{
  float currentAltitude = barometer.readAltitude(1013.25) - initialAltitude;
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);
  float currentMagnitude = sqrt(pow(mainIMUCurrAccelAxes[0], 2) + pow(mainIMUCurrAccelAxes[1], 2) + pow(mainIMUCurrAccelAxes[2], 2));

  return ((currentMagnitude - initialMagnitude) > 0 || currentAltitude > 0);
}

/*
 * @brief Checks if rocket is in APOGEE phase.
 *
 * Ensures that either acceleration or altitude is decreasing to verify maximum altitude
 *
 * @return Boolean value: true if data is successfully verified transitions to APOGEE phase, false if conditions arent met.
 *
 */
bool CheckApogeeConditions()
{

  float currentAltitude = barometer.readAltitude(1013.25) - initialAltitude;

  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);
  float currentMagnitude = sqrt(pow(mainIMUCurrAccelAxes[0], 2) +
                                pow(mainIMUCurrAccelAxes[1], 2) +
                                pow(mainIMUCurrAccelAxes[2], 2));

  // 0.5m minimum altitude threshold hardcoded to detect descent
  bool isDescending = (currentAltitude < lastAltitude - 0.5);
  // 0.2 minimum deceleration threshold hardcoded to detect descent
  bool isDecelerating = (currentMagnitude - initialMagnitude) < -0.2;

  lastAltitude = currentAltitude;
  lastAccel = currentMagnitude;

  return (isDescending && isDecelerating);
}