#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// arduino nano

#define SEALEVELPRESSURE_HPA (1011.75) // MAKE SURE to change to configure altitude to correct LEVELS EVERYTIME

#define RX_PIN 3 // Connect to TX of RYLR998
#define TX_PIN 2 // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);
uint8_t readStatus = 0;
unsigned int timer = 0;

void setup()
{
  Serial.begin(9600);     // Serial Monitor
  loraSerial.begin(9600); // LoRa module baud rate

  loraSerial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  loraSerial.println("AT+ADDRESS=1"); // Sets radio address to 1
  delay(1000);

  loraSerial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  loraSerial.println("AT+IPR=9600"); // Sets baud rate at 9600
  delay(1000);                       // Allow module to initialize
}

void loop()
{

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;
  float humidity = myAHT10.readHumidity(AHT10_USE_READ_DATA);
  String buffer;
  buffer = String(altitude) + "," + String(humidity) + "," + String(timer);
  timer++;
  String command = "AT+SEND=2," + String(buffer.length()) + "," + buffer;
  loraSerial.println(command);

  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);

  delay(1000

  );
}