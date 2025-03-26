#include <Arduino.h>

const int RPWM=2;
const int LPWM=3;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);

  delay(1000);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) { // Check if there is data in the buffer
    String input = Serial.readString(); // Read the incoming string
    input.trim(); // Remove any leading/trailing whitespace

    if (input.length() > 0 && input.toInt() != 0 || input == "0") { 
      int speedInt = input.toInt(); // Convert to integer

      float speed = constrain(speedInt / 100.0, -1.0, 1.0); // Normalize if needed
      if (speed > 0.0) {
        analogWrite(RPWM, speed * 255);
        analogWrite(LPWM, 0);
      } else if (speed < 0.0) {
        analogWrite(LPWM, -speed * 255);
        analogWrite(RPWM, 0);
      } else {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
      }

      Serial.print("Speed set to: ");
      Serial.println(speed);
    } else {
      Serial.println("Invalid input. Please send an integer.");
    }
  }
}