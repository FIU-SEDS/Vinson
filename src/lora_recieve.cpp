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
  Serial.begin(9600);     // Serial Monitor
  loraSerial.begin(9600); // LoRa module baud rate

  loraSerial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  loraSerial.println("AT+ADDRESS=2"); // Sets radio address to 1
  delay(1000);

  loraSerial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  loraSerial.println("AT+IPR=9600"); // Sets baud rate at 9600
  delay(1000);                       // Allow module to initialize

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
