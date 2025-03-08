#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define CHIP_SELECT 10 // SD Card Chip Select Pin

File logFile;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    ; // Wait for Serial Monitor to open (for debugging on boards like Leonardo)
  }

  Serial.println("Initializing SD card...");

  if (!SD.begin(CHIP_SELECT))
  {
    Serial.println("SD initialization failed!");
    while (1)
      ; // Halt execution if SD card fails
  }

  Serial.println("SD card initialized successfully.");
}

void logData()
{
  logFile = SD.open("data.txt", FILE_WRITE);

  if (logFile)
  {
    String dataString = "Time: " + String(millis()) + " ms";
    logFile.println(dataString);
    logFile.close();
    Serial.println("Logged: " + dataString);
  }
  else
  {
    Serial.println("Error opening data.txt");
  }
}

void loop()
{
  logData();
  delay(1000); // Log data every second
}
