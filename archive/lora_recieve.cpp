#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>

// arduino mega

#define RX_PIN 10 // Connect to TX of RYLR998
#define TX_PIN 11 // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);

void setup()
{
  Serial.begin(115200);     // Serial Monitor
  loraSerial.begin(115200); // LoRa module baud rate

  delay(1000);
  Serial.println("LoRa Receiver Ready!");
}

void loop()
{
  if (loraSerial.available())
  { // Check if data is received
    String receivedData = loraSerial.readString();
    Serial.println("Received: " + receivedData); // Print to Serial Monitor
  }
}
