#include <Arduino.h>
#include <Wire.h>
#include "SensorManager.h"

RocketState currentState = INIT_AND_SYSTEMS_CHECK;

void setup()
{
  Serial.begin(115200);
  // Initialize I2C bus.
  DEV_I2C.begin();
  Wire.begin();

  InitializeLoRa();

  // Perform one-time sensor checks
  if (!InitializeAndCheckSensors())
  {
    Serial.println("[CRITICAL] One or more critical sensors failed. Halting...");
    // while (true)
    //   ; // Halt on failure
  }

  // Transition to IDLE state for liftoff monitoring
  currentState = IDLE;
  Serial.println("[INFO] System ready. Awaiting liftoff.");
}

void loop()
{
  StartData(currentState);

  switch (currentState)
  {
  case INIT_AND_SYSTEMS_CHECK:
    // this is just a GHOST case to avoid any compiler warnings but this section of the code should typically be skipped as it is handled in the setup() function
    break;

  case IDLE:
    if (CheckLiftoffConditions())
    {
      currentState = BOOST;
      Serial.println("[STATE] Transitioning to BOOST phase.");
    }
    break;

  case BOOST:
    if (CheckApogeeConditions())
    {
      currentState = APOGEE;
      Serial.println("[STATE] Transitioning to APOGEE phase.");
    }
    break;

  case APOGEE:
    if (CheckDrogueDeployment())
    {
      currentState = DROGUE_DEPLOY;
    }
    break;

  case DROGUE_DEPLOY:
    DeployDrogueParachute();
    currentState = MAIN_DEPLOY;
    break;

  case MAIN_DEPLOY:

    if (CheckLandingConditions())
    {
      currentState = LANDED;
    }
    break;

  case LANDED:
    DumpData(); // TODO Uses SD card to save all data and stop writing

    // implement variable or function that stops the continuing writing of data
    break;
  }
}