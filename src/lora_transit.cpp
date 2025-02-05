#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// arduino nano

#define SEALEVELPRESSURE_HPA (1013.25)

#define RX_PIN 5 // Connect to TX of RYLR998
#define TX_PIN 6 // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);
Adafruit_BMP3XX bmp;

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

  if (!bmp.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
    // if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    // if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop()
{
  // loraSerial.println("AT+SEND=2,10,HELLOWORLD"); // Send data
  // delay(1000);
  // Serial.println("SENT"); // Wait 2 seconds before sending again

  if (!bmp.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  char buffer[10];                 // Buffer to store formatted float as string
  dtostrf(altitude, 6, 2, buffer); // Convert float to string with 2 decimal places

  loraSerial.println("AT+SEND=2,10," + String(buffer));

  Serial.println("SENT");
  delay(2000);
}
