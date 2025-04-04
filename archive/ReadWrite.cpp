/*
  SD card read/write

  This example shows how to read and write data to and from an SD card file
  The circuit. Pin numbers reflect the default
  SPI pins for Uno and Nano models:
   SD card attached to SPI bus as follows:
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (For For Uno, Nano: pin 10. For MKR Zero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified  24 July 2020
  by Tom Igoe

  This example code is in the public domain.

*/
#include <SD.h>
#include <SoftwareSerial.h>

#define RX_PIN 3 // Connect to TX of RYLR998
#define TX_PIN 2 // Connect to RX of RYLR998
#define CHIP_SELECT 10

SoftwareSerial loraSerial(RX_PIN, TX_PIN);
File myFile;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  loraSerial.begin(9600);
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial)
    ;
  loraSerial.println("AT+MODE=0"); // Sets radio to transciever mode
  delay(1000);

  loraSerial.println("AT+ADDRESS=1"); // Sets radio address to 1
  delay(1000);

  loraSerial.println("AT+BAND=915000000"); // sets bandwith to 915 Mhz
  delay(1000);

  loraSerial.println("AT+IPR=9600"); // Sets baud rate at 9600
  delay(1000);                       // Allow module to initialize

  Serial.println("LoRa Transmitter Ready!");

  Serial.print("Initializing SD card...");

  if (!SD.begin(CHIP_SELECT))
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true)
      ;
  }

  Serial.println("initialization done.");
}

void loop()
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile)
  {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
    String command = "AT+SEND=2," + String(5) + "," + "HELLO";
    loraSerial.println(command);
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  // // re-open the file for reading:
  // myFile = SD.open("test.txt");
  // if (myFile)
  // {
  //   Serial.println("test.txt:");

  //   // read from the file until there's nothing else in it:
  //   while (myFile.available())
  //   {
  //     Serial.write(myFile.read());
  //   }
  //   // close the file:
  //   myFile.close();
  // }
  // else
  // {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening test.txt");
  // }
}
