#include <Arduino.h>

#define EN     6      //Stepper Motor Enable, Active Low Level 
#define DIR    7      //Stepper Motor Direction Control 
#define STP    8      //Stepper Control
#define LIMIT_SWITCH_1 9
#define LIMIT_SWITCH_2 10

#define SLOW_SPEED 10000
#define FAST_SPEED 1000

float currentPos = 0;
float maxPos = 0;

// Move the stepper motor a certain number of steps (positive or negative)
void step(int steps) {
  if (steps > 0) {
    digitalWrite(DIR, LOW);
  } else {
    digitalWrite(DIR, HIGH);
  }
  for (int i = 0; i < steps; i++) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(FAST_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(FAST_SPEED);
    currentPos += 1;
    if (currentPos > maxPos) {
      return;
    }
  }
  delayMicroseconds(100);
}

void home() {
  // Move in the negative direction until the limit switch is pressed
  digitalWrite(DIR, HIGH);
  while (digitalRead(LIMIT_SWITCH_1) == HIGH) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
  }
  currentPos = 0;
  delayMicroseconds(100);
  // Move in the positive direction until the limit switch is pressed
  digitalWrite(DIR, LOW);
  while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
    maxPos += 1;
  }
  delayMicroseconds(100);
}

void setup() {
  pinMode(DIR, OUTPUT);
  pinMode(STP, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

  digitalWrite(EN, LOW);
  Serial.setTimeout(200);
  Serial.begin(115200);

  delay(500);
  Serial.println("Homing...");
  home();
  Serial.println("Homing complete. Max position: " + String(maxPos));
}

void loop() {
  int distanceStr = Serial.readStringUntil('\n').toInt();
  if (distanceStr == 0) {
    return;
  }
  int distance = distanceStr;
  Serial.println("Moving " + String(distance) + " steps...");
  step(distance);
}