#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>

#define RX_PIN 2  // Connect to TX of RYLR998
#define TX_PIN 3  // Connect to RX of RYLR998

SoftwareSerial loraSerial(RX_PIN, TX_PIN);

void setup() {
    Serial.begin(115200);     // Serial Monitor
    loraSerial.begin(115200); // LoRa module baud rate

    delay(1000);  // Allow module to initialize
}

void loop() {
    loraSerial.println("AT+SEND=0,20,asdasdasdasdasdasdas"); // Send data
    delay(100);  // Wait 2 seconds before sending again
}