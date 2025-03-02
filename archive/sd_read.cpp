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

  byte buffer[512];
  SDCore::read(0, buffer); // Read block 0

  // Print CSV data (can be copied to a file)
  Serial.println((char *)buffer);
}

void loop() {}