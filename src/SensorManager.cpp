#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <ASM330LHHSensor.h> // Main IMU Library

bool criticalSensors[1];

// Sensor objects983MA magnetometer;
ASM330LHHSensor mainIMU(&Wire, ASM330LHH_I2C_ADD_L);

// Arrays to hold accelerometer readings
int32_t mainIMUCurrAccelAxes[3] = {}; // For current acceleration (x, y, z)

// Variables that will change over time
float initialAltitude = 0.0; // Initial altitude reference
float lastAltitude = 0.0;    // Last known altitude

// Flags and time tracking variables
// For DeployDrogueParachute()28084
unsigned long deployStartTime = 0; // Time tracking for deployment
unsigned long apogeeStartTime = 0; // Variable to store the last time update was made

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
#if DEBUG
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
#endif
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
#if DEBUG
    logStatus("Main IMU", "I2C Start Device Check (Power Up Function)", false);
#endif
    return false;
  }

  if (mainIMU.begin() != ASM330LHH_OK)
  {
#if DEBUG
    logStatus("Main IMU", "Power Up", false);
#endif
    return false;
  }

  mainIMU.Enable_X();
  mainIMU.Enable_G();
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  if (mainIMU.verifyTemperature() != ASM330LHH_OK)
  {
#if DEBUG
    logStatus("Main IMU", "Temperature Check", false);
#endif
    return false;
  }

#if DEBUG
  logStatus("Main IMU", "Power Up", true);
#endif

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

#if DEBUG
  // Log the status of critical sensors
  Serial.println("[INFO] Critical Sensor Status:");
  Serial.print("  Main IMU: ");
  Serial.println(criticalSensors[MAIN_IMU] ? "SUCCESS" : "FAILURE");
  Serial.print("  Barometer: ");
  Serial.println(criticalSensors[BAROMETER] ? "SUCCESS" : "FAILURE");
#endif

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
 *
 *  */
bool CheckLiftoffConditions()
{
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Check if acceleration exceeds 1.5g (1.5g = 1500 mg)
  return (mainIMUCurrAccelAxes[Z] > LIFTOFF_GRAVITY_THRESHOLD);
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
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Check if the Z-axis acceleration is close to 0g (freefall condition at apogee)
  bool isDecelerating = (mainIMUCurrAccelAxes[Z] < APOGEE_GRAVITY_THRESHOLD); // Z-axis acceleration (in mg)

  unsigned long currentMillis = millis();

  // Check if 1 second has passed
  if ((currentMillis - apogeeStartTime) >= INTERVAL_APOGEE)
  {
    // Save the last time you updated
    apogeeStartTime = currentMillis;

    // Code to run after 1 second has passed
    CheckApogeeConditions(); // For example, check apogee conditions
  }

  return (isDecelerating);
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
#if DEBUG
    logStatus("Drogue Parachute", "Deployment Conditions", true);
#endif

    return true;
  }

#if DEBUG
  logStatus("Drogue Parachute", "Deployment Conditions", false);
#endif
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
    // DONT USE DELAY USE MILLIS()
    // Short delay to ensure the signal is registered
    // digitalWrite(DROGUE_DEPLOY_PIN, LOW);

#if DEBUG
    logStatus("Drogue Parachute", "Signal Sent DEPLOYING", true);
#endif
    drogueDeployed = true;
  }
  else
  {
#if DEBUG
    logStatus("Drogue Parachute", "Already Deployed", true);
#endif
  }
}

/**
 * @brief Checks if the rocket has landed based on acceleration and altitude.
 *
 * @return Boolean value: true if the rocket is considered landed, false otherwise.
 */
bool CheckLandingConditions()
{
  // Read current accelerometer data
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

// Log the current values for debugging (if necessary)
#if DEBUG
  logStatus("Landing Check", "Acceleration Magnitude", accelerationMagnitude);
#endif
  // Check if the acceleration magnitude is near zero (indicating no significant movement)
  bool isStationary = (mainIMUCurrAccelAxes[Z] <= LANDING_GRAVITY_THRESHOLD);

  // If both conditions are met, the rocket is considered landed
  if (isStationary)
  {
#if DEBUG
    logStatus("Landing Check", "Landing Detected", true);
#endif
    return true;
  }

#if DEBUG
  logStatus("Landing Check", "Not Landed", false);
#endif

  return false;
}