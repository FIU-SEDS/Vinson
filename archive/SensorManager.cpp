#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> // Magnetometer Library
#include <ASM330LHHSensor.h>                    // Main IMU Library
#include <Adafruit_BMP3XX.h>                    // Barometer Library
#include "Adafruit_HTU21DF.h"                   // HTU Library


bool criticalSensors[2];
bool nonCriticalSensors[2];

// Sensor objects983MA magnetometer;
ASM330LHHSensor mainIMU(&Wire, ASM330LHH_I2C_ADD_L);
Adafruit_BMP3XX barometer;
Adafruit_HTU21DF HTU = Adafruit_HTU21DF();
SFE_MMC5

// Arrays to hold accelerometer readings
int32_t mainIMUInitAccelAxes[3] = {}; // For initial acceleration (x, y, z)
int32_t mainIMUCurrAccelAxes[3] = {}; // For current acceleration (x, y, z)

// Variables that will change over time
float initialAltitude = 0.0;           // Initial altitude reference
float initialAcceleration = 0.0;       // Initial acceleration reference
float initialMagnitude = 0.0;          // Initial acceleration magnitude
float initialAmbientTemperature = 0.0; // Initial ambient temperature on ground
float initialRelativeHumidity = 0.0;   // Initial relative humidity on ground
float lastAltitude = 0.0;              // Last known altitude
float lastAccel = 0.0;                 // Last known acceleration

// Flags and time tracking variables
// For DeployDrogueParachute()
bool deployStarted = false;        // Flag to indicate deployment started
unsigned long deployStartTime = 0; // Time tracking for deployment

/**
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

/**
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

// *******************************************  MAGNETOMETER INIT *******************************************

/**
 * @brief Powers up the Magnetometer sensor.
 *
 * This function checks the I2C connection to the Magnetometer. If connected, it initializes the
 * magnetometer, performs a soft reset, and verifies the temperature. Logs the status of the power-up process.
 *
 * @return Boolean value: true if the Magnetometer is successfully powered up, false otherwise.
 */
bool PowerMagnetometer()
{
  if (!isDeviceConnected(MAGNETOMETER_ADDRESS))
    logStatus("Magnetometer", "I2C Start Device Check (Power Up Function)", false);
  return false;

  if (!magnetometer.begin())
  {
    logStatus("Magnetometer", "Power Up", false);
    return false;
  }
  magnetometer.softReset();

  if (!magnetometer.verifyTemperature())
  {
    logStatus("Magnetometer", "Temperature Check", false);
    return false;
  }

  logStatus("Magnetometer", "Power Up", true);
  return true;
}

/**
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
  {
    logStatus("Main IMU", "I2C Start Device Check (Power Up Function)", false);
    return false;
  }

  if (mainIMU.begin() != ASM330LHH_OK)
  {
    logStatus("Main IMU", "Power Up", false);
    return false;
  }

  mainIMU.Enable_X();
  mainIMU.Enable_G();
  mainIMU.Get_X_Axes(mainIMUInitAccelAxes);
  initialMagnitude = sqrt(pow(mainIMUInitAccelAxes[0], 2) + pow(mainIMUInitAccelAxes[1], 2) + pow(mainIMUInitAccelAxes[2], 2));

  if (mainIMU.verifyTemperature() != ASM330LHH_OK)
  {
    logStatus("Main IMU", "Temperature Check", false);
    return false;
  }

  logStatus("Main IMU", "Power Up", true);
  return true;
}

// *******************************************  BAROMETER INIT *******************************************

/**
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
  {
    logStatus("Barometer", "I2C Start Device Check (Power Up Function)", false);
    return false;
  }

  if (!barometer.begin_I2C(BAROMETER_ADDRESS))
  {
    logStatus("Barometer", "Power Up", false);
    return false;
  }

  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);
  initialAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * 3.28084; // using initialAltitude from .h file to set the sea-level altitude in FEET as baseline

  if (!barometer.verifyTemperature())
  {
    logStatus("Barometer", "Temperature Check", false);
    return false;
  }

  logStatus("Barometer", "Power Up", true);
  return true;
}

/**
 * @brief Powaers up the HTU20DF sensor.
 *
 * This function checks the I2C connection to the HTU20DF sensor. If connected, it initializes the
 * sensor and reads the initial ambient temperature and relative humidity. The status of the
 * power-up process is logged.
 *
 * @return Boolean value: true if the HTU20DF sensor is successfully powered up, false otherwise.
 */

bool PowerHTU()
{
  if (!isDeviceConnected(HTU21DF_I2CADDRESS))
  {
    logStatus("HTU", "I2C Start Device Check (Power Up Function)", false);
    return false;
  }

  if (!HTU.begin())
  {

    logStatus("HTU", "Power Up", false);
    return false;
  }

  initialAmbientTemperature = HTU.readTemperature(); // returned in celsius
  initialRelativeHumidity = HTU.readHumidity();

  logStatus("HTU", "Power Up", true);
  return true;
}

/**
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

  criticalSensors[MAIN_IMU] = PowerMainIMU();
  criticalSensors[BAROMETER] = PowerBarometer();
  nonCriticalSensors[MAGNETOMETER] = PowerMagnetometer();
  nonCriticalSensors[HTURHT] = PowerHTU();

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
  Serial.print(" HTU: ");
  Serial.println(nonCriticalSensors[HTURHT] ? "SUCCESS" : "FAILURE");

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

/**
 * @brief Checks if rocket is in BOOST phase.
 *
 * Ensures that either acceleration or altitude is increasing to verify successful rocket launch
 *
 * @return Boolean value: true if data is successfully verified transitions to BOOST phase, false if conditions arent met.
 *
 */
bool CheckLiftoffConditions()
{
  float currentAltitude = (barometer.readAltitude(SEA_LEVEL_PRESSURE) * 3.28084) - initialAltitude;

  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);
  float currentMagnitude = sqrt(pow(mainIMUCurrAccelAxes[0], 2) + pow(mainIMUCurrAccelAxes[1], 2) + pow(mainIMUCurrAccelAxes[2], 2));

  return ((currentMagnitude - initialMagnitude) > 0 || currentAltitude > 0);
}

/**
 * @brief Checks if rocket is in APOGEE phase.
 *
 * Ensures that either acceleration or altitude is decreasing to verify maximum altitude
 *
 * @return Boolean value: true if data is successfully verified transitions to APOGEE phase, false if conditions arent met.
 *
 */
bool CheckApogeeConditions()
{

  float currentAltitude = (barometer.readAltitude(SEA_LEVEL_PRESSURE) * 3.28084) - initialAltitude;

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

// NOTE: The static keyword in these functions is used to declare variables whose values persist across multiple calls to the function, rather than being reinitialized each time the function is invoked. (so every time the function is called it is not reset to 0 rather it keeps its value)

/**
 * @brief Checks if five seconds have passed since apogee to deploy drogue parachutes.
 *
 * @return Boolean value: true if five seconds passed and deployment signal sent,
 * false if conditions aren't met.
 */
bool CheckDrogueDeployment()
{
  static unsigned long apogeeTime = 0;

  if (apogeeTime == 0)
  {
    apogeeTime = millis(); // Record the time at apogee if not already set
  }

  if (millis() - apogeeTime >= 5000) // Check if 5 seconds have passed
  {
    logStatus("Drogue Parachute", "Deployment Conditions", true);
    return true;
  }

  logStatus("Drogue Parachute", "Deployment Conditions", false);
  return false;
}

/**
 * @brief Deploys the drogue parachute by sending a signal to the deployment mechanism.
 *
 * @details This function triggers the drogue deployment mechanism, ensuring the
 * signal is sent only once. Includes logging for debugging purposes.
 */
void DeployDrogueParachute()
{
  static bool drogueDeployed = false;

  if (!drogueDeployed)
  {
    // UNCOMMENT WHEN DETERMINED PINS LOCATION

    // digitalWrite(DROGUE_DEPLOY_PIN, HIGH); // Activiates pyro mechanism for drogue parachute
    delay(100); // Short delay to ensure the signal is registered
    // digitalWrite(DROGUE_DEPLOY_PIN, LOW);

    logStatus("Drogue Parachute", "Signal Sent DEPLOYING", true);
    drogueDeployed = true;
  }
  else
  {
    logStatus("Drogue Parachute", "Already Deployed", true);
  }
}

/**
 * @brief Checks if the rocket's altitude is at or below the main deployment altitude.
 *
 * @return Boolean value: true if the altitude is at or below the main deployment threshold, false otherwise.
 */
bool CheckMainDeploymentConditions()
{
  float currentAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * 3.28084; // Converted to feet

  // Check if the rocket has descended to or below the main deployment altitude
  if (currentAltitude <= MAIN_DEPLOYMENT_ALTITUDE)
  {
    logStatus("Main Parachute", "Deployment Conditions", true);
    return true;
  }

  logStatus("Main Parachute", "Deployment Conditions", false);
  return false;
}

/**
 * @brief Deploys the main parachute by sending a signal to the deployment mechanism.
 *
 * @details Sends a single signal to deploy the main parachute. Uses minimal memory
 * and ensures the signal is sent only once by relying on external state management.
 */
void DeployMainParachute()
{
  static bool mainDeployed = false;

  if (!mainDeployed)
  {
    // UNCOMMENT WHEN DETERMINED PINS LOCATION

    // digitalWrite(MAIN_DEPLOY_PIN, HIGH); // Activiates pyro mechanism for drogue parachute
    delay(100); // Short delay to ensure the signal is registered
    // digitalWrite(MAIN_DEPLOY_PIN, LOW);

    logStatus("Main Parachute", "Signal Sent DEPLOYING", true);
    mainDeployed = true;
  }
  else
  {
    logStatus("Main Parachute", "Already Deployed", true);
  }
}

/**
 * @brief Checks if the rocket has landed based on acceleration and altitude.
 *
 * @return Boolean value: true if the rocket is considered landed, false otherwise.
 */
bool CheckLandingConditions()
{

  // Read current altitude from the barometer (convert to feet or meters as needed)
  float currentAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * 3.28084; // Convert to feet if needed

  // Read current accelerometer data
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Calculate the magnitude of the current acceleration (use existing variables directly)
  float accelerationMagnitude = sqrt(pow(mainIMUCurrAccelAxes[0], 2) + pow(mainIMUCurrAccelAxes[1], 2) + pow(mainIMUCurrAccelAxes[2], 2));

  // Log the current values for debugging (if necessary)
  logStatus("Landing Check", "Current Altitude", currentAltitude);
  logStatus("Landing Check", "Acceleration Magnitude", accelerationMagnitude);

  // Check if the altitude is close to ground level (within a small tolerance)
  bool altitudeAtGround = (currentAltitude <= LANDING_ALTITUDE);

  // Check if the acceleration magnitude is near zero (indicating no significant movement)
  bool isStationary = (accelerationMagnitude <= ACCELERATION_THRESHOLD);

  // If both conditions are met, the rocket is considered landed
  if (altitudeAtGround && isStationary)
  {

    logStatus("Landing Check", "Landing Detected", true);
    return true;
  }

  logStatus("Landing Check", "Not Landed", false);
  return false;
}
