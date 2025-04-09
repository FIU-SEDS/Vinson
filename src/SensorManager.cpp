#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <ASM330LHHSensor.h> // Main IMU Library
#include <SD.h>
#include <SPI.h>
// #include <SoftwareSerial.h>

bool criticalSensors[1];

// Sensor objects983MA magnetometer;
ASM330LHHSensor mainIMU(&DEV_I2C, ASM330LHH_I2C_ADD_L);
File logFile;
// SoftwareSerial loraSerial(RX_PIN, TX_PIN);

// Arrays to hold accelerometer readings
int32_t mainIMUCurrAccelAxes[3] = {}; // For current acceleration (x, y, z)
int32_t mainIMUCurrGyroAxes[3] = {};  // For current acceleration (x, y, z)

// Flags and time tracking variables
// For DeployDrogueParachute()28084
unsigned long deployStartTime = 0; // Time tracking for deployment
unsigned long apogeeStartTime = 0; // Variable to store the last time update was made
unsigned long startMillis;         // some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long interval = 300;
unsigned int timer = 0;           // Flight Time Elapsed
bool sdCardAvailable = false;   // SD card detection
bool dataCollectionActive = true; // SD card flag to stop saving data

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

void InitializeLoRa()
{
  Serial.begin(115200); // LoRa module baud rate

  Serial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  Serial.println("AT+ADDRESS=1"); // Sets radio address to 1
  delay(1000);

  Serial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  Serial.println("AT+IPR=115200"); // Sets baud rate at 115200
  delay(1000);                     // Allow module to initialize

  Serial.println("LoRa Transmitter Ready!");

  pinMode(CHIP_SELECT_PIN, OUTPUT); // Make sure CS pin is set as output
  sdCardAvailable = SD.begin(CHIP_SELECT_PIN);

  if (sdCardAvailable)
  {
    Serial.println("SD initialization successful!");
  }
  else
  {
    Serial.println("SD card not detected. Continuing without logging.");
  }

  startMillis = millis();
}

void StartData(RocketState current_state)
{

  // If data collection has been stopped, don't collect or transmit
  if (!dataCollectionActive)
  {
    return;
  }

  // Only process data at specified intervals
  currentMillis = millis();
  if (currentMillis - startMillis >= interval)
  {
    startMillis = currentMillis;
    timer++;

    // Getting sensor data
    mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);
    mainIMU.Get_G_Axes(mainIMUCurrGyroAxes);

    // Build data string
    String buffer;
    buffer = String(mainIMUCurrAccelAxes[0]) + "," + String(mainIMUCurrAccelAxes[1]) + "," + String(mainIMUCurrAccelAxes[2]) + "," + String(mainIMUCurrGyroAxes[0]) + "," + String(mainIMUCurrGyroAxes[1]) + "," + String(mainIMUCurrGyroAxes[2]) + "," + String(timer) + "," + String(current_state);

    // Create LoRa command
    String command = "AT+SEND=2," + String(buffer.length()) + "," + buffer;

    // Log to SD card if available
    if (sdCardAvailable)
    {
      logFile = SD.open("flight.txt", FILE_WRITE);
      if (logFile)
      {
        logFile.println(command);
        logFile.close();
      }
      else
      {
        Serial.println("Error opening data.txt");
      }
    }

    // Always send data over LoRa regardless of SD card status
    Serial.println(command);
  }
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
  mainIMU.Get_G_Axes(mainIMUCurrGyroAxes);

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
#endif

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

#if DEBUG
  Serial.println("Aceeleration: [mg]: ");
  Serial.print(mainIMUCurrAccelAxes[X]);
  Serial.println(" X");
  Serial.print(mainIMUCurrAccelAxes[Y]);
  Serial.println(" Y");
  Serial.print(mainIMUCurrAccelAxes[Z]);
  Serial.println(" Z");

#endif

  // Check if acceleration exceeds Liftoff threshold value
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
  bool isNearMicrogravity = (abs(mainIMUCurrAccelAxes[Z]) < APOGEE_GRAVITY_THRESHOLD); // Z-axis acceleration (in mg)

#if DEBUG
  Serial.print("Z acceleration at apogee check: ");
  Serial.print(mainIMUCurrAccelAxes[Z]);
  Serial.print(" mg, Near microgravity: ");
  Serial.println(isNearMicrogravity ? "YES" : "NO");
#endif

  return isNearMicrogravity;
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
  static int stableReadingCount = 0;

  // Read current accelerometer data
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);

  // Check if acceleration is within the landing thresholds
  bool isLandingAcceleration = (mainIMUCurrAccelAxes[Z] > LANDING_GRAVITY_THRESHOLD_LOW &&
                                mainIMUCurrAccelAxes[Z] < LANDING_GRAVITY_THRESHOLD_HIGH);

  if (isLandingAcceleration)
  {
    stableReadingCount++;
  }
  else
  {
    stableReadingCount = 0; // Reset the counter if readings are outside the landing range
  }

#if DEBUG
  if (stableReadingCount > 0)
  {
    Serial.print("Landing check: Z acceleration: ");
    Serial.print(mainIMUCurrAccelAxes[Z]);
    Serial.print(" mg, Stable readings: ");
    Serial.print(stableReadingCount);
    Serial.print("/");
    Serial.println(LANDING_SAMPLE_COUNT);
  }
#endif

  // Consider landed if we have enough consistent readings within the landing range
  if (stableReadingCount >= LANDING_SAMPLE_COUNT)
  {
#if DEBUG
    Serial.println("[STATE] Landing confirmed!");
#endif
    return true;
  }

  return false;
}

/**
 * @brief Finalizes data collection and prepares for shutdown
 *
 * This function is called when the rocket has landed. It ensures all
 * data is properly saved to the SD card, transmits a final status message,
 * and prepares the system for shutdown or low-power mode.
 */
void DumpData()
{
  static bool dataFinalized = false;

  // Only run this once
  if (!dataFinalized)
  {
    // Stop data collection
    dataCollectionActive = false;

    // Transmit final status
    String finalStatus = "AT+SEND=2,14,FLIGHT COMPLETE";
    Serial.println(finalStatus);

    // Save final status if SD card available
    if (sdCardAvailable)
    {
      // Write a final entry with conclusion timestamp
      logFile = SD.open("flight.txt", FILE_WRITE);
      if (logFile)
      {
        logFile.println("FLIGHT SUMMARY");
        logFile.print("Total flight time (s): ");
        logFile.println(timer * (interval / 1000.0)); // Convert to seconds
        logFile.println("Flight state: LANDED");
        logFile.println("Data collection complete");
        logFile.println(finalStatus);
        logFile.println("END OF DATA");
        logFile.close();
      }
      else
      {
        Serial.println("Error opening flight.txt for final entry");
      }
    }

    // Set flag to indicate data has been finalized
    dataFinalized = true;

    // Send confirmation message
    Serial.println("Data collection finalized, safe to power off");
  }
}
