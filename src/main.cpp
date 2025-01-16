#include <SPI.h>
#include <SD.h>
#include <Adafruit_I2CDevice.h>

const int chipSelect = 10; // Pin for CS (Chip Select)

void setup()
{
  // Start the serial communication
  Serial.begin(115200);

  // Initialize SPI and SD card
  if (!SD.begin(chipSelect))
  {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
}

void loop()
{
  // Example: Write to a file
  File dataFile = SD.open("test.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println("Hello, SD card!");
    dataFile.close();
    Serial.println("Data written to SD card.");
  }
  else
  {
    Serial.println("Error opening file.");
  }
  delay(1000);
}
