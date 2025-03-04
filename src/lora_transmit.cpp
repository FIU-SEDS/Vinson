#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <AHT10.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// arduino nano

#define SEALEVELPRESSURE_HPA (1018) // MAKE SURE to change to configure altitude to correct LEVELS EVERYTIME

#define RX_PIN 5 // Connect to TX of RYLR998
#define TX_PIN 6 // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);
Adafruit_BMP3XX bmp;
uint8_t readStatus = 0;
unsigned int timer = 0;
AHT10 myAHT10(AHT10_ADDRESS_0X38);

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

  if (!myAHT10.begin())
  {
    Serial.println("Failed to initialize AHT10 sensor");
  }
  else
  {
    Serial.println("AHT10 initialized");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop()
{
  // loraSerial.println("AT+SEND=2,10,HELLOWORLD"); // Send data
  // delay(1000);
  // Serial.println("SENT"); // Wait 2 seconds before sending agains

  if (!bmp.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }

  readStatus = myAHT10.readRawData(); // Read 6 bytes from AHT10 over I2C
  if (readStatus != AHT10_ERROR)
  {
    Serial.print("AHT10 Humidity: ");
    Serial.print(myAHT10.readHumidity(AHT10_USE_READ_DATA)); // Use previously read 6 bytes
  }
  else
  {
    Serial.println("AHT10 I2C error");
  }

  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;
  float humidity = myAHT10.readHumidity(AHT10_USE_READ_DATA);
  String buffer;
  buffer = String(altitude) + "," + String(humidity) + "," + String(timer);
  timer++;
  String command = "AT+SEND=2," + String(buffer.length()) + "," + buffer;
  loraSerial.println(command);

  Serial.println("SENT");
  delay(2000);
}