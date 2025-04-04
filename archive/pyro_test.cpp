#include <Wire.h>
#include <Arduino.h>

#define PYRO_PIN 9

void setup()
{
  delay(10000);
  pinMode(PYRO_PIN, OUTPUT);

  digitalWrite(PYRO_PIN, HIGH);
  delay(100);
  digitalWrite(PYRO_PIN, LOW);
  delay(100);
}

void loop()
{
}