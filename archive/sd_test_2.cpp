#include <Arduino.h>
#include <SDCore.h>

// Global variables to track block usage
uint32_t currentBlock = 0;       // Start with block 0
uint16_t currentBlockOffset = 0; // Current position within block
byte blockBuffer[512] = {0};     // Buffer for the current block
bool bufferModified = false;     // Track if buffer has been modified

// Function prototypes
void writeToSD(const char *data);
void flushBuffer();
void clearBlock(uint32_t blockNumber);

void setup()
{
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // Wait 2 seconds
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

  // Optional: Start at a specific block number
  currentBlock = 0; // Change this if you want to start at a different block

  // Load the current block to see if there's already data
  SDCore::read(currentBlock, blockBuffer);

  // Find the end of existing data in this block
  while (currentBlockOffset < 512 && blockBuffer[currentBlockOffset] != 0)
  {
    currentBlockOffset++;
  }

  Serial.print("Starting at block ");
  Serial.print(currentBlock);
  Serial.print(" offset ");
  Serial.println(currentBlockOffset);

  // Test: write some data
  const char *csvData1 = "0.25,1013.2,50.2\n";
  writeToSD(csvData1);

  const char *csvData2 = "0.34,1013.5,52.4\n";
  writeToSD(csvData2);

  // Important: make sure any buffered data is written
  flushBuffer();

  // Read back data for verification
  Serial.println("\nVerifying data:");
  byte readBuffer[512] = {0};

  SDCore::read(currentBlock, readBuffer);
  Serial.print("Block ");
  Serial.print(currentBlock);
  Serial.println(": ");
  Serial.println((char *)readBuffer);

  // Finish SD Card communication
  SDCore::end();
}

void loop()
{
  // Optional: Add data collection in loop
  // static unsigned long lastSampleTime = 0;
  // static unsigned long lastFlushTime = 0;

  // // Collect data every X milliseconds
  // if (millis() - lastSampleTime > 1000) { // Every second
  //   lastSampleTime = millis();
  //
  //   // Create sample data - replace with your actual sensor readings
  //   char buffer[50];
  //   float temp = random(2000, 3000) / 100.0;  // 20.00-30.00
  //   float pressure = 1013.0 + random(-50, 50) / 10.0;
  //   float altitude = 50.0 + random(-10, 10);
  //
  //   sprintf(buffer, "%.2f,%.1f,%.1f\n", temp, pressure, altitude);
  //   writeToSD(buffer);
  // }
  //
  // // Flush data to SD card periodically to prevent data loss
  // if (millis() - lastFlushTime > 5000) { // Every 5 seconds
  //   lastFlushTime = millis();
  //   flushBuffer();
  // }
}

// Write data to SD card, handling block boundaries
void writeToSD(const char *data)
{
  size_t dataLength = strlen(data);
  size_t bytesWritten = 0;

  while (bytesWritten < dataLength)
  {
    // Calculate how much data we can fit in the current block
    size_t bytesAvailable = 512 - currentBlockOffset;
    size_t bytesToWrite = min(bytesAvailable, dataLength - bytesWritten);

    // Copy data to the buffer
    memcpy(blockBuffer + currentBlockOffset, data + bytesWritten, bytesToWrite);
    currentBlockOffset += bytesToWrite;
    bytesWritten += bytesToWrite;
    bufferModified = true;

    // If block is full or close to full, write it and move to next block
    if (currentBlockOffset >= 512)
    {
      SDCore::write(currentBlock, blockBuffer);
      Serial.print("Block ");
      Serial.print(currentBlock);
      Serial.println(" is full, moving to next block");

      // Move to next block
      currentBlock++;
      currentBlockOffset = 0;

      // Clear buffer for new block
      memset(blockBuffer, 0, 512);
      bufferModified = false;
    }
  }
}

// Ensure any buffered data is written to the SD card
void flushBuffer()
{
  if (bufferModified)
  {
    SDCore::write(currentBlock, blockBuffer);
    Serial.print("Flushed partially filled block ");
    Serial.println(currentBlock);
    bufferModified = false;
  }
}

// Function to clear a specific block (fill with zeros)
void clearBlock(uint32_t blockNumber)
{
  // Create a temporary buffer filled with zeros
  byte zeroBuffer[512] = {0};

  // Write zeros to the specified block
  bool success = SDCore::write(blockNumber, zeroBuffer);

  // Report status
  Serial.print("Clearing block ");
  Serial.print(blockNumber);

  if (success)
  {
    Serial.println(" - Success");

    // If we're clearing the current working block, reset our buffer too
    if (blockNumber == currentBlock)
    {
      memset(blockBuffer, 0, 512);
      currentBlockOffset = 0;
      bufferModified = false;

      Serial.println("Reset current buffer");
    }
  }
  else
  {
    Serial.println(" - Failed");
  }
}