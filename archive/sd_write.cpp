#include <Arduino.h>
#include <SDCore.h>

uint32_t current_block = 0; // Start with block 0

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
  // const char *csvData = "0.25,1013.2,50.2\n";
  // const char *csvData01 = "0.34,1013.5,52.4\n";

  // Copy data into the buffer (ensuring it's within 512 bytes)
  // strcpy((char *)buffer, csvData);
  // strcat((char *)buffer, csvData01);
  for (uint16_t i = 1; i <= 512; i++)
  {
    strcat((char *)buffer, "H");
  }

  bool status = SDCore::write(current_block, buffer);
  Serial.print("Status: ");
  Serial.println(status);

  SDCore::read(current_block, buffer); // Read block 0

  // Serial.println("Buffer Length (using sizeof): ");
  // Serial.println(sizeof(buffer));

  Serial.println("Buffer Length: ");
  Serial.println(strlen((char *)buffer));
  // Print CSV data (can be copied to a file)
  Serial.println((char *)buffer);

  memset(buffer, 0, 512); // Clear buffer
  // For Block 2 as we already passed the 512 byte buffer
  strcpy((char *)buffer, "F");
  strcat((char *)buffer, "F");
  strcat((char *)buffer, "F");

  current_block++;

  status = SDCore::write(current_block, buffer);
  Serial.print("Status: ");
  Serial.println(status);

  SDCore::read(current_block, buffer);
  Serial.println((char *)buffer + 512);

  // // Write to current block
  // if (strlen((char *)buffer) <= 512)
  // {
  //   Serial.println("BUFFER 512 PASSED NEXT BLOCK READ");
  //   current_block++;
  //   SDCore::read(current_block, buffer);

  //   Serial.println((char *)buffer + 512);
  // }
}

void loop()
{
}