#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(1); // wait a maximum of 1ms for incoming serial data
}

void loop()
{
  String message;

  while (!Serial.available()) // wait for incoming serial data when available
  {
    message = Serial.readString(); // copies incoming string
    message.toUpperCase();
    Serial.println(message);
  }
}