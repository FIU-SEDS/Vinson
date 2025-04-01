// LORA TRANSMIT ACCEL,GYRESCOPE,SDCARD SAVE

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <ASM330LHHSensor.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define RX_PIN 3 // Connect to TX of RYLR998
#define TX_PIN 2 // Connect to RX of RYLR998
#define CHIP_SELECT 10

SoftwareSerial loraSerial(RX_PIN, TX_PIN);
ASM330LHHSensor main_IMU(&DEV_I2C, ASM330LHH_I2C_ADD_L);
SFE_MMC5983MA magnetometer;
uint8_t readStatus = 0;
unsigned int timer = 0;

unsigned long startMillis; // some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long interval = 1500;
int second = 0;

enum RocketState
{
  LIFTOFF,
  BOOST,
  APOGEE
};

RocketState currentState;
File file;

void StartData()
{
  String command = "AT+SEND=0,2,HI";
  Serial.println(command);
}

void InitializeLoRa()
{

  Serial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  Serial.println("AT+NETWORK=0");
  delay(1000);

  Serial.println("AT+ADDRESS=1"); // Sets radio address to 1
  delay(1000);

  Serial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  Serial.println("AT+IPR=115200"); // Sets baud rate at 9600
  delay(1000);                     // Allow module to initialize
}

void setup()
{
  Serial.begin(115200); // Serial Monitor
  Wire.begin();

  InitializeLoRa();
}

void loop()
{
  StartData();
  delay(300);
}
