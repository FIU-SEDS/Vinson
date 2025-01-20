#include <StoreData.h>

void setup()
{
  SPI.begin();

  // Initialize SD card
  if (!SD.begin(CS))
  {
    Serial.println("SD Card initialization failed!");
    return; // Exit if SD card initialization fails
  }

  // Open the file on the SD card for writing in append mode
  myFile = SD.open("payload.txt", FILE_WRITE);

  // Check if the file opened successfully
  if (myFile)
  {
    Serial.println("File opened successfully for writing.");
    Serial.println("Writing to Payload.txt...");
  }
  else
  {
    Serial.println("Error opening the file.");
  }
}

void loop()
{
}
