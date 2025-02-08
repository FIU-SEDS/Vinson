#include <Arduino.h>
#include <SPI.h>
#include "BMP390.h"

#define SEALEVEL 

BMP390 bmp;


void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing BMP390...");

  if (!bmp.begin())
  {
    Serial.println("BMP390 not found! Check wiring.");
    while (1)
      ;
  }
  Serial.println("BMP390 initialized successfully!");

  bmp.set_temperature_oversampling(2);
  bmp.set_pressure_oversampling(2);
  bmp.set_IIR_filter_coeff(0);
  bmp.set_output_data_rate(50);
}

void loop()
{
  Serial.print("Temperature: ");
  Serial.print(bmp.read_temperature());
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(bmp.read_pressure());
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(bmp.read_altitude(101325)); // Assume sea-level pressure is 1013.25 hPa
  Serial.println(" m");

  Serial.println("-------------------------");
  delay(1000);
}
