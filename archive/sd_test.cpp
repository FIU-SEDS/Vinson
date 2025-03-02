#include <Arduino.h>
#include <SDCore.h>

void setup()
{
  // Initialize serial communication at 9600 baund
  Serial.begin(9600);

  // Wait a couple of seconds (optional)
  // Allow you to get the serial comm active
  delay(2000);

  // Initialize SD Card
  byte response = SDCore::begin();

  if (!response)
  {
    Serial.println("SD Initialization: FAILED");
    Serial.println("Make sure than you have your wiring correct and a compatible SD card inserted.");
    return;
  }
  Serial.println("SD Initialization: SUCCESS");

  byte buffer[512] = {0}; // Empty block buffer

  // Create CSV-style data (e.g., "Temperature,Pressure,Altitude")
  const char *csvData = "0.25,1013.2,50.2\n";
  const char *csvData01 = "0.34,1013.5,52.4\n";

  // Copy data into the buffer (ensuring it's within 512 bytes)
  strcpy((char *)buffer, csvData);
  strcat((char *)buffer, csvData01);

  // Write to block 0
  bool status = SDCore::write(0, buffer);
  Serial.println("Status: ");
  Serial.println(status);

 
  SDCore::read(0, buffer); // Read block 0

  // Print CSV data (can be copied to a file)
  Serial.println((char *)buffer);
}

void loop()
{
}