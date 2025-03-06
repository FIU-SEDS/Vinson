#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <ASM330LHHSensor.h> // Main IMU Library
#include <Adafruit_BMP3XX.h> // Barometer Library

bool criticalSensors[2];

// Sensor objects983MA magnetometer;
ASM330LHHSensor mainIMU(&Wire, ASM330LHH_I2C_ADD_L);
Adafruit_BMP3XX barometer;

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
#if DEBUG
    logStatus("Barometer", "I2C Start Device Check (Power Up Function)", false);
#endif
    return false;
  }

  if (!barometer.begin_I2C(BAROMETER_ADDRESS))
  {
#if DEBUG
    logStatus("Barometer", "Power Up", false);
#endif
    return false;
  }

  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!barometer.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return false;
  }

  // initialAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER; // using initialAltitude from .h file to set the sea-level altitude in FEET as baseline

  // Serial.print("Initial Altitude: ");
  // Serial.print(initialAltitude);
  // Serial.println(" ft");

  for (uint8_t i = 0; i < 2; i++)
  {
    Serial.print("Approx. Altitude = ");
    Serial.print(barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER);
    Serial.println(" ft ");

    if(i == 1)
    {
      initialAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER;
    }
  }

  if (!barometer.verifyTemperature())
  {
#if DEBUG
    logStatus("Barometer", "Temperature Check", false);
#endif
    return false;
  }

#if DEBUG
  logStatus("Barometer", "Power Up", true);
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

  // criticalSensors[MAIN_IMU] = PowerMainIMU();
  criticalSensors[MAIN_IMU] = true;
  criticalSensors[BAROMETER] = PowerBarometer();

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
  float currentAltitude = (barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER) - initialAltitude;

  if (!barometer.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return false;
  }
  
  Serial.print("Initial Altitude: ");
  Serial.print(initialAltitude);
  Serial.println(" ft");

  Serial.print("Current. Altitude = ");
  Serial.print(currentAltitude);
  Serial.println(" ft ");

  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Check if acceleration exceeds 1.5g (1.5g = 1500 mg)
  return ((mainIMUCurrAccelAxes[Z] > LIFTOFF_GRAVITY_THRESHOLD) || currentAltitude > LIFTOFF_ALTITUDE_THRESHOLD);
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

  if (!barometer.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return false;
  }

  // initialAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER; // using initialAltitude from .h file to set the sea-level altitude in FEET as baseline

  // Serial.print("Initial Altitude: ");
  // Serial.print(initialAltitude);
  // Serial.println(" ft");

  float currentAltitude = (barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER) - initialAltitude;

  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Check if altitude is descending (using 1 ft as the minimum descent threshold)
  bool isDescending = (currentAltitude < lastAltitude - APOGEE_ALTITUDE_THRESHOLD);

  // Check if the Z-axis acceleration is close to 0g (freefall condition at apogee)
  bool isDecelerating = (mainIMUCurrAccelAxes[Z] < APOGEE_GRAVITY_THRESHOLD); // Z-axis acceleration (in mg)

  lastAltitude = currentAltitude;

  unsigned long currentMillis = millis();

  // Check if 1 second has passed
  if (currentMillis - apogeeStartTime >= INTERVAL_APOGEE)
  {
    // Save the last time you updated
    apogeeStartTime = currentMillis;

    // Code to run after 1 second has passed
    CheckApogeeConditions(); // For example, check apogee conditions
  }

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
 * @brief Checks if the rocket's altitude is at or below the main deployment altitude.
 *
 * @return Boolean value: true if the altitude is at or below the main deployment threshold, false otherwise.
 */
bool CheckMainDeploymentConditions()
{
  float currentAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER; // Converted to feet

  // Check if the rocket has descended to or below the main deployment altitude
  if (currentAltitude <= MAIN_DEPLOYMENT_ALTITUDE)
  {
#if DEBUG
    logStatus("Main Parachute", "Deployment Conditions", true);
#endif

    return true;
  }

#if DEBUG
  logStatus("Main Parachute", "Deployment Conditions", false);
#endif
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
    // DONT USE DELAY USE MILLIS()
    // Short delay to ensure the signal is registered
    // digitalWrite(MAIN_DEPLOY_PIN, LOW);

#if DEBUG
    logStatus("Main Parachute", "Signal Sent DEPLOYING", true);
#endif
    mainDeployed = true;
  }
  else
  {
#if DEBUG
    logStatus("Main Parachute", "Already Deployed", true);
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

  // Read current altitude from the barometer (convert to feet or meters as needed)
  float currentAltitude = barometer.readAltitude(SEA_LEVEL_PRESSURE) * FEET_PER_METER; // Convert to feet if needed

  // Read current accelerometer data
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

// Log the current values for debugging (if necessary)
#if DEBUG
  logStatus("Landing Check", "Current Altitude", currentAltitude);
  logStatus("Landing Check", "Acceleration Magnitude", accelerationMagnitude);
#endif
  // Check if the altitude is close to ground level (within a small tolerance)
  bool altitudeAtGround = (currentAltitude <= LANDING_ALTITUDE);

  // Check if the acceleration magnitude is near zero (indicating no significant movement)
  bool isStationary = (mainIMUCurrAccelAxes[Z] <= LANDING_GRAVITY_THRESHOLD);

  // If both conditions are met, the rocket is considered landed
  if (altitudeAtGround && isStationary)
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