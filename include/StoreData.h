#ifndef STORE_DATA_H
#define STORE_DATA_H

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Arduino.h>

#define CS 10

Adafruit_BMP3XX bmp; // I2C
File myFile;

void SD_Card_Setup()
{
  SPI.begin();

  // Initialize SD card
  if (!SD.begin(CS))
  {
    Serial.println("SD Card initialization failed!");
    return; // Exit if SD card initialization fails
  }

  // Open the file on the SD card for writing in append mode
  myFile = SD.open("payload.txt", FILE_WRITE);

  // Check if the file opened successfully
  if (myFile)
  {
    Serial.println("File opened successfully for writing.");
    Serial.println("Writing to Payload.txt...");
  }
  else
  {
    Serial.println("Error opening the file.");
  }
}

void SD_Card_Upload()
{

  if (myFile)
  {
    myFile.print("Temperature: ");
    myFile.print(bmp.readTemperature());
    myFile.println(" C*");

    myFile.print("Pressure:  ");
    myFile.print(bmp.readPressure());
    myFile.println("  Pa");

    myFile.flush();

    Serial.println("Data Log Successful!");

    delay(3000);
  }
}

void read_sensor_data()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.println();
  delay(3000);
}

#endif