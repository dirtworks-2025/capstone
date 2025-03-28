#include <SPI.h>

// Use this code to test the SPI communication with the nRF24L01+ module
// This does not require the RF24 library and is a simple test to check if the
// nRF24L01+ is responding to SPI commands.

const int CE_PIN = 7;    // CE (not needed for this test but good practice to define)
const int CSN_PIN = 8;  // Chip select (CSN)

byte readRegister(byte reg) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0x00 | (reg & 0x1F)); // Read command
  byte result = SPI.transfer(0xFF);  // Dummy write to receive data
  digitalWrite(CSN_PIN, HIGH);
  return result;
}

void setup() {
  Serial.begin(9600);

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);
  digitalWrite(CSN_PIN, HIGH);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); // slower
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  delay(100); // Allow power-up time

  byte config = readRegister(0x00); // CONFIG register
  Serial.print("CONFIG register: 0x");
  Serial.println(config, HEX);

  if (config == 0x08 || config == 0x0B || config == 0x0F) {
    Serial.println("nRF24L01+ is alive!");
  } else {
    Serial.println("nRF24L01+ not responding (check wiring and power!)");
  }
}

void loop() {
  // Nothing to do in loop
}
