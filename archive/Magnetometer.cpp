/*
  X/Y/Z magnetic field and raw readings from the MMC5983MA
  By: Nathan Seidle and Ricardo Ramos
  SparkFun Electronics
  Date: April 14th, 2022
  License: SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/19034

  This example demonstrates how to read the basic X/Y/Z readings from the sensor over Qwiic

  Hardware Connections:
  Plug a Qwiic cable into the sensor and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper
  (https://www.sparkfun.com/products/17912) Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

SFE_MMC5983MA myMag;

void setup()
{
    Serial.begin(9600);
    Serial.println("MMC5983MA Example");

    Wire.begin();

    if (myMag.begin() == false)
    {
        Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
        while (true)
            ;
    }

    myMag.softReset();

    Serial.println("MMC5983MA connected");

    int celsius = myMag.getTemperature();
    float fahrenheit = (celsius * 9.0f / 5.0f) + 32.0f;

    Serial.print("Die temperature: ");
    Serial.print(celsius);
    Serial.print("°C or ");
    Serial.print(fahrenheit, 0);
    Serial.println("°F.");
}

void loop()
{
    uint32_t currentX = 0;
    uint32_t currentY = 0;
    uint32_t currentZ = 0;
    double scaledX = 0;
    double scaledY = 0;
    double scaledZ = 0;

    // // This reads the X, Y and Z channels consecutively
    // // (Useful if you have one or more channels disabled)
    // currentX = myMag.getMeasurementX();
    // currentY = myMag.getMeasurementY();
    // currentZ = myMag.getMeasurementZ();

    // Or, we could read all three simultaneously
    myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);

    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
    // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss.
    //
    // Please note: to properly correct and calibrate the X, Y and Z channels, you need to determine true
    // offsets (zero points) and scale factors (gains) for all three channels. Futher details can be found at:
    // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
    scaledX = (double)currentX - 131072.0;
    scaledX /= 131072.0;
    scaledY = (double)currentY - 131072.0;
    scaledY /= 131072.0;
    scaledZ = (double)currentZ - 131072.0;
    scaledZ /= 131072.0;

    // The magnetometer full scale is +/- 8 Gauss
    // Multiply the scaled values by 8 to convert to Gauss
    Serial.print("X axis field (Gauss): ");
    Serial.print(scaledX * 8, 5); // Print with 5 decimal places

    Serial.print("\tY axis field (Gauss): ");
    Serial.print(scaledY * 8, 5);

    Serial.print("\tZ axis field (Gauss): ");
    Serial.println(scaledZ * 8, 5);

    Serial.println();
    delay(100);
}
