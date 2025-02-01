#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>

#define LED_PIN 11

void setup()
{
  pinMode(LED_PIN, OUTPUT);
}

void loop()

{
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}