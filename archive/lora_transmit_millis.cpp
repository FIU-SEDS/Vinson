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
  // file = SD.open("test.txt", FILE_WRITE);
  // if (file)
  // {
  //   Serial.print("Writing to test.txt...");
  //   file.println("testing 1, 2, 3.");
  //   // close the file:
  //   file.close();
  //   Serial.println("done.");
  // }
  // else
  // {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening test.txt");
  // }

  // Read accelerometer and gyroscope.
  currentMillis = millis();

  if (currentMillis - startMillis >= interval)
  {
    startMillis = millis();
    timer++;

    int32_t accelerometer[3] = {};
    int32_t gyroscope[3] = {};
    main_IMU.Get_X_Axes(accelerometer);
    main_IMU.Get_G_Axes(gyroscope);

    String buffer;
    buffer = String(accelerometer[0]) + "," + String(accelerometer[1]) + "," + String(accelerometer[2]) + "," + String(gyroscope[0]) + "," + String(gyroscope[1]) + "," + String(gyroscope[2]) + "," + String(timer);

    String command = "AT+SEND=2," + String(buffer.length()) + "," + buffer;
    loraSerial.println(command);
    Serial.println("SENT");
  }
}

void InitializeLoRa()
{

  loraSerial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  loraSerial.println("AT+ADDRESS=1"); // Sets radio address to 1
  delay(1000);

  loraSerial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  loraSerial.println("AT+IPR=115200"); // Sets baud rate at 9600
  delay(1000);                       // Allow module to initialize

  Serial.println("LoRa Transmitter Ready!");
  startMillis = millis();
}

void setup()
{
  Serial.begin(115200);     // Serial Monitor
  loraSerial.begin(115200); // LoRa module baud rate
  DEV_I2C.begin();
  Wire.begin();

  InitializeLoRa();

  // if (!SD.begin(CHIP_SELECT))
  // {
  //   Serial.println("initialization failed. Things to check:");
  //   Serial.println("1. is a card inserted?");
  //   Serial.println("2. is your wiring correct?");
  //   Serial.println("3. did you change the chipSelect pin to match your shield or module?");
  //   Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  //   while (true)
  //     ;
  // }

  // Serial.println("SD initialization done!");

  // Initilizing Magnetometer
  if (magnetometer.begin() == false)
  {
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
  }

  magnetometer.softReset();

  // Initilize ASM330LHH Sensor
  main_IMU.begin();
  main_IMU.Enable_X();
  main_IMU.Enable_G();

  currentState = LIFTOFF;
}

void loop()
{
  StartData();

  switch (currentState)
  {
  case LIFTOFF:
    /* code */
    // Serial.println("LIFTOFF");
    break;
  default:
    break;
  }
}
