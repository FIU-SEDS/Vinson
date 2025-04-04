#include "SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <ASM330LHHSensor.h>

// =========== KALMAN FILTER IMPLEMENTATION ===========

int32_t mainIMUCurrAccelAxes[3] = {}; // For current acceleration (x, y, z)
int32_t mainIMUCurrGyroAxes[3] = {};  // For current acceleration (x, y, z)

ASM330LHHSensor mainIMU;

unsigned long startMillis; // some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long interval = 300;
// Flight Time Elapsed
unsigned int timer = 0;

// Kalman filter variables for accelerometer
float accel_estimate[3] = {0, 0, 0};       // Current estimated values
float accel_uncertainty[3] = {10, 10, 10}; // Current uncertainty in estimates
float accel_filtered[3] = {0, 0, 0};       // Filtered output values

// Kalman filter variables for gyroscope
float gyro_estimate[3] = {0, 0, 0};       // Current estimated values
float gyro_uncertainty[3] = {10, 10, 10}; // Current uncertainty in estimates
float gyro_filtered[3] = {0, 0, 0};       // Filtered output values

// Kalman filter parameters - tune these as needed
float ACCEL_MEASUREMENT_NOISE = 8.0; // Higher = trust sensors less (for noisy environments)
float ACCEL_PROCESS_NOISE = 2.0;     // Higher = more responsive to changes
float GYRO_MEASUREMENT_NOISE = 1.0;  // Higher = trust sensors less
float GYRO_PROCESS_NOISE = 0.3;      // Higher = more responsive to changes

/**
 * @brief Processes accelerometer and gyroscope data through Kalman filter
 *
 * Applies a Kalman filter to each axis of the accelerometer and gyroscope
 * data to reduce noise while preserving important signal changes.
 *
 * @param raw_accel Array containing raw accelerometer data [X, Y, Z]
 * @param raw_gyro Array containing raw gyroscope data [X, Y, Z]
 */
void updateKalmanFilter(int32_t raw_accel[3], int32_t raw_gyro[3])
{
  // Convert int32_t data to float for filter processing
  float accel_input[3], gyro_input[3];

  for (int i = 0; i < 3; i++)
  {
    // Convert int32_t to float (maintain the same scale/units)
    accel_input[i] = (float)raw_accel[i];
    gyro_input[i] = (float)raw_gyro[i];

    // Process accelerometer data
    // Step 1: Prediction (increase uncertainty)
    accel_uncertainty[i] = accel_uncertainty[i] + ACCEL_PROCESS_NOISE;

    // Step 2: Calculate Kalman gain (how much to trust the measurement)
    float accel_k_gain = accel_uncertainty[i] / (accel_uncertainty[i] + ACCEL_MEASUREMENT_NOISE);

    // Step 3: Update estimate with weighted measurement
    accel_estimate[i] = accel_estimate[i] + accel_k_gain * (accel_input[i] - accel_estimate[i]);

    // Step 4: Update uncertainty
    accel_uncertainty[i] = (1 - accel_k_gain) * accel_uncertainty[i];

    // Store filtered value
    accel_filtered[i] = accel_estimate[i];

    // Process gyroscope data
    // Step 1: Prediction
    gyro_uncertainty[i] = gyro_uncertainty[i] + GYRO_PROCESS_NOISE;

    // Step 2: Calculate Kalman gain
    float gyro_k_gain = gyro_uncertainty[i] / (gyro_uncertainty[i] + GYRO_MEASUREMENT_NOISE);

    // Step 3: Update estimate
    gyro_estimate[i] = gyro_estimate[i] + gyro_k_gain * (gyro_input[i] - gyro_estimate[i]);

    // Step 4: Update uncertainty
    gyro_uncertainty[i] = (1 - gyro_k_gain) * gyro_uncertainty[i];

    // Store filtered value
    gyro_filtered[i] = gyro_estimate[i];
  }
}

/**
 * @brief Tunes the Kalman filter parameters
 *
 * Allows adjustment of the filter behavior to match specific flight conditions:
 * - Higher measurement noise = smoother but slower response
 * - Higher process noise = more responsive but potentially noisier
 *
 * @param accel_meas_noise How much to trust accelerometer readings (higher = trust less)
 * @param accel_proc_noise How much accelerometer values change between readings
 * @param gyro_meas_noise How much to trust gyroscope readings (higher = trust less)
 * @param gyro_proc_noise How much gyroscope values change between readings
 */
void tuneKalmanFilter(float accel_meas_noise, float accel_proc_noise,
                      float gyro_meas_noise, float gyro_proc_noise)
{
  ACCEL_MEASUREMENT_NOISE = accel_meas_noise;
  ACCEL_PROCESS_NOISE = accel_proc_noise;
  GYRO_MEASUREMENT_NOISE = gyro_meas_noise;
  GYRO_PROCESS_NOISE = gyro_proc_noise;
}

/**
 * @brief Resets the Kalman filter if needed
 *
 * Useful after significant events like landing impacts or
 * when filter values might be far from actual readings.
 */
void resetKalmanFilter()
{
  for (int i = 0; i < 3; i++)
  {
    accel_estimate[i] = 0;
    gyro_estimate[i] = 0;
    accel_uncertainty[i] = 10;
    gyro_uncertainty[i] = 10;
  }
}

// =========== MODIFIED SENSOR FUNCTIONS ===========

/**
 * @brief Reads sensor data and applies Kalman filtering
 *
 * Gets raw sensor values from the ASM330LHH IMU and
 * applies the Kalman filter to smooth the data
 */
void readAndFilterSensorData()
{
  // Read raw sensor data
  mainIMU.Get_X_Axes(mainIMUCurrAccelAxes);
  mainIMU.Get_G_Axes(mainIMUCurrGyroAxes);

  // Apply Kalman filter
  updateKalmanFilter(mainIMUCurrAccelAxes, mainIMUCurrGyroAxes);

#if DEBUG
  Serial.println("Raw Acceleration [mg]: ");
  Serial.print(mainIMUCurrAccelAxes[X]);
  Serial.print(", ");
  Serial.print(mainIMUCurrAccelAxes[Y]);
  Serial.print(", ");
  Serial.print(mainIMUCurrAccelAxes[Z]);

  Serial.println("\nFiltered Acceleration [mg]: ");
  Serial.print(accel_filtered[X]);
  Serial.print(", ");
  Serial.print(accel_filtered[Y]);
  Serial.print(", ");
  Serial.print(accel_filtered[Z]);

  Serial.println("\nRaw Gyro [mdps]: ");
  Serial.print(mainIMUCurrGyroAxes[X]);
  Serial.print(", ");
  Serial.print(mainIMUCurrGyroAxes[Y]);
  Serial.print(", ");
  Serial.print(mainIMUCurrGyroAxes[Z]);

  Serial.println("\nFiltered Gyro [mdps]: ");
  Serial.print(gyro_filtered[X]);
  Serial.print(", ");
  Serial.print(gyro_filtered[Y]);
  Serial.print(", ");
  Serial.print(gyro_filtered[Z]);
  Serial.println("\n");
#endif
}

// =========== MODIFIED FLIGHT CONDITION CHECKS ===========

/**
 * @brief Checks if rocket is in BOOST phase using filtered data
 *
 * Uses Kalman-filtered acceleration to determine if liftoff has occurred
 *
 * @return Boolean value: true if liftoff conditions are met
 */
bool CheckLiftoffConditions()
{
  readAndFilterSensorData();

  // Use filtered Z-axis acceleration instead of raw
  return (accel_filtered[Z] > LIFTOFF_GRAVITY_THRESHOLD);
}

/**
 * @brief Checks if rocket is at APOGEE phase using filtered data
 *
 * Uses Kalman-filtered acceleration to determine if apogee has been reached
 *
 * @return Boolean value: true if apogee conditions are met
 */
bool CheckApogeeConditions()
{
  readAndFilterSensorData();

  // Use filtered Z-axis acceleration
  bool isDecelerating = (accel_filtered[Z] < APOGEE_GRAVITY_THRESHOLD);

  return isDecelerating;
}

/**
 * @brief Checks if the rocket has landed based on filtered acceleration
 *
 * @return Boolean value: true if the rocket is considered landed
 */
bool CheckLandingConditions()
{
  readAndFilterSensorData();

  // Check if the acceleration is near gravity (indicating no significant movement)
  bool isStationary = (accel_filtered[Z] <= LANDING_GRAVITY_THRESHOLD);

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

// =========== MODIFIED DATA TRANSMISSION ===========

void StartData(RocketState current_state)
{
  currentMillis = millis();

  if (currentMillis - startMillis >= interval)
  {
    startMillis = currentMillis;
    timer++;

    // Read and filter sensor data
    readAndFilterSensorData();

    // Send both raw and filtered data
    String buffer;
    buffer = String(mainIMUCurrAccelAxes[0]) + "," +
             String(mainIMUCurrAccelAxes[1]) + "," +
             String(mainIMUCurrAccelAxes[2]) + "," +
             String(mainIMUCurrGyroAxes[0]) + "," +
             String(mainIMUCurrGyroAxes[1]) + "," +
             String(mainIMUCurrGyroAxes[2]) + "," +
             String((int)accel_filtered[0]) + "," +
             String((int)accel_filtered[1]) + "," +
             String((int)accel_filtered[2]) + "," +
             String((int)gyro_filtered[0]) + "," +
             String((int)gyro_filtered[1]) + "," +
             String((int)gyro_filtered[2]) + "," +
             String(timer) + "," +
             String(current_state);

    String command = "AT+SEND=2," + String(buffer.length()) + "," + buffer;
    Serial.println(command);
  }
}