#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

void setup() {
  Serial.begin(115200);   // Start serial communication
  while (!Serial)         // Wait for serial monitor
    ;
  Serial.println("\nSPI Scanner");
  
  SPI.begin();            // Initialize SPI
}

void loop() {
  byte devicesFound = 0;

  Serial.println("Scanning...");

  // Try to communicate with SPI devices by asserting the chip select
  for (byte chipSelectPin = 10; chipSelectPin <= 10; chipSelectPin++) {  // Pin 10 as the chip select pin
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);  // Deselect the chip

    // Try each address by asserting the chip select
    digitalWrite(chipSelectPin, LOW);   // Select the chip
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Set SPI parameters

    byte response = SPI.transfer(0x00); // Send dummy byte to get a response

    SPI.endTransaction(); // End SPI transaction

    if (response != 0xFF) { // Check if a response is received (assuming 0xFF means no device)
      Serial.print("SPI device found at CS pin: ");
      Serial.println(chipSelectPin);
      devicesFound++;
    }

    digitalWrite(chipSelectPin, HIGH);  // Deselect the chip
  }

  if (devicesFound == 0) {
    Serial.println("No SPI devices found\n");
  } else {
    Serial.println("Done\n");
  }

  delay(5000); // Wait 5 seconds before the next scan
}
