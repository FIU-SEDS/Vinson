#include <SPI.h>
#include <Adafruit_I2CDevice.h>

const int chipSelect = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial Monitor
  }

  Serial.println("Starting SPI test...");

  SPI.begin();
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH); // Deactivate CS initially

  // Test with slower clock and specific SPI mode
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  digitalWrite(chipSelect, LOW); // Activate the device
  delay(10);

  Serial.println("Sending test command...");
  uint8_t response = SPI.transfer(0x55); // Send a different test byte

  digitalWrite(chipSelect, HIGH); // Deactivate the device
  delay(10);

  Serial.print("Response from device: 0x");
  Serial.println(response, HEX);

  SPI.endTransaction();
  SPI.end();
}

void loop() {
  // No need to loop for this test
}
