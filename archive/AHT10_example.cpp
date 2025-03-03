
#include <Wire.h>
#include <AHT10.h>

#define LCD_ROWS 4         // qnt. of lcd rows
#define LCD_COLUMNS 20     // qnt. of lcd columns
#define DEGREE_SYMBOL 0xDF // degree symbol from LCD ROM

uint8_t readStatus = 0;

const uint8_t temperature_icon[8] PROGMEM = {0x04, 0x0A, 0x0A, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E}; // PROGMEM saves variable to flash & keeps dynamic memory free
const uint8_t humidity_icon[8] PROGMEM = {0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x00};
const uint8_t plus_minus_icon[8] PROGMEM = {0x00, 0x04, 0x0E, 0x04, 0x00, 0x0E, 0x00, 0x00};

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
