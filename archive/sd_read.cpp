#include <Arduino.h>
#include <SDCore.h>

// Global variables to track block usage
uint32_t current_block = 0;        // Start with block 0
uint16_t current_block_offset = 0; // Current position within block
byte block_buffer[512] = {0};      // Buffer for the current block
bool bufferModified = false;       // Track if buffer has been modified

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

  current_block = 0; // Start at specific block

  SDCore::read(current_block, block_buffer); // Read block 0

  Serial.print("Buffer Length: ");
  Serial.println(strlen((char *)block_buffer));

  // Find the end of existing data in this block
  while (current_block_offset < 512 && block_buffer[current_block_offset] != 0)
  {
    current_block_offset++;
  }

  Serial.print("Starting at block ");
  Serial.print(current_block);
  Serial.print(" offset ");
  Serial.println(current_block_offset);

  // Print CSV data (can be copied to a file)
  Serial.println((char *)block_buffer);

  if (strlen((char *)block_buffer) >= 512)
  {
    Serial.println("BUFFER 512 PASSED NEXT BLOCK READ");

    current_block++;          // Move to the next block
    current_block_offset = 0; // Reset the offset

    if (SDCore::read(current_block, block_buffer)) // Check if read is successful
    {

      // Find the end of existing data in this block
      while (current_block_offset < 512 && block_buffer[current_block_offset] != 0)
      {
        current_block_offset++;
      }

      Serial.print("Reading block ");
      Serial.print(current_block);
      Serial.print(" offset ");
      Serial.println(current_block_offset);

      Serial.println((char *)block_buffer);
    }
    else
    {
      Serial.println("Failed to read next block.");
    }
  }
}

void loop() {}