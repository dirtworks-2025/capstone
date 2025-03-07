// Test code for stepper motor
// Using DM542T stepper motor driver
// Using NEMA 23 stepper motor

#include <Arduino.h>

const int ENABLE=6;
const int DIR=7;
const int STEP=8;

void setup() {
  pinMode(ENABLE, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);

  digitalWrite(ENABLE, LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(STEP, LOW);
  delay(1000);
  Serial.begin(9600);
}

void loop() {
    delayMicroseconds(5);
    digitalWrite(STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP, LOW);
}