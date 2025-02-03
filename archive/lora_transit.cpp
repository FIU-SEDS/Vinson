#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>

// arduino nano

#define RX_PIN 5 // Connect to TX of RYLR998
#define TX_PIN 6 // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);

void setup()
{
    Serial.begin(9600);     // Serial Monitor
    loraSerial.begin(9600); // LoRa module baud rate

    loraSerial.println("AT+IPR-9600");
    delay(1000); // Allow module to initialize
}

void loop()
{
    loraSerial.println("AT+SEND=2,10,HELLOWORLD"); // Send data
    delay(1000);
    Serial.println("SENT"); // Wait 2 seconds before sending again
}