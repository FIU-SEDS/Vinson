#include <Wire.h>
#include <AHT10.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

uint8_t readStatus = 0;
AHT10 myAHT10(AHT10_ADDRESS_0X38);
Adafruit_BMP3XX bmp;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  // Wait for Serial Monitor to be open
  Serial.println("Starting Sensor Initialization");

  // BMP390 initialization
  if (!bmp.begin_I2C())
  {
    Serial.println("Failed to initialize BMP390 sensor");
  }
  else
  {
    Serial.println("BMP390 initialized");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // AHT10 initialization
  if (!myAHT10.begin())
  {
    Serial.println("Failed to initialize AHT10 sensor");
  }
  else
  {
    Serial.println("AHT10 initialized");
  }
}

void loop()
{
  // Read AHT10 data
  readStatus = myAHT10.readRawData(); // Read 6 bytes from AHT10 over I2C
  if (readStatus != AHT10_ERROR)
  {
    Serial.print("AHT10 Humidity: ");
    Serial.print(myAHT10.readHumidity(AHT10_USE_READ_DATA)); // Use previously read 6 bytes
    Serial.println(" %");
  }
  else
  {
    Serial.println("AHT10 I2C error");
  }

  // Read BMP390 data
  if (bmp.performReading())
  {
    Serial.print("BMP390 Pressure: ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("BMP390 Altitude: ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
  }
  else
  {
    Serial.println("BMP390 I2C error");
  }

  delay(3000); // Delay to avoid flooding serial output
}
