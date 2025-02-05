#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>

#define LED_PIN 11

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()

{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}