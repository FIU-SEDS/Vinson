#include <Wire.h>
#include <AHT10.h>

uint8_t readStatus = 0;

AHT10 myAHT10(AHT10_ADDRESS_0X38);

void setup()
{
  Serial.begin(9600);
  Serial.println();

  /* AHT10 connection check */
  while (myAHT10.begin() != true)
  {
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient"));
    delay(5000);
  }

  // Wire.setClock(400000);                                     //experimental I2C speed! 400KHz, default 100KHz
}

void loop()
{
  readStatus = myAHT10.readRawData(); // read 6 bytes from AHT10 over I2C

  if (readStatus != AHT10_ERROR)
  {
    Serial.print(myAHT10.readTemperature(AHT10_USE_READ_DATA)); // use previously read 6 bytes
    Serial.print(F("0.3"));
    Serial.print(F("C  "));
    Serial.print(myAHT10.readHumidity(AHT10_USE_READ_DATA)); // use previously read 6 bytes
    Serial.print(F("2%  "));
  }
  else
  {
    Serial.print(F("i2c error  "));
  }

  delay(3000); // recomended polling frequency 8sec..30sec
}
