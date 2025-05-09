#include <Arduino.h>

#define EN     6      //Stepper Motor Enable, Active Low Level 
#define DIR    7      //Stepper Motor Direction Control 
#define STP    8      //Stepper Control
#define LIMIT_SWITCH_1 9
#define LIMIT_SWITCH_2 10

#define SLOW_SPEED 5000
#define FAST_SPEED 2000

const float IN_PER_STEP = 0.025; // Estimated distance per step (TODO: measure this and update)

float currentPos = 0; // Inches
float maxPos = 0;     // Inches

// Move the stepper motor a certain number of steps (positive or negative)
void gantryMove(int steps) {
  digitalWrite(DIR, (steps > 0) ? HIGH : LOW);
  for (int i = 0; i < abs(steps); i++) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(FAST_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(FAST_SPEED);
    currentPos += IN_PER_STEP * (steps > 0 ? 1 : -1);
    if (digitalRead(LIMIT_SWITCH_1) == LOW || digitalRead(LIMIT_SWITCH_2) == LOW) {
      Serial.println("Error: Limit switch pressed.");
      return;
    }
  }
  delayMicroseconds(100);
}

void gantryGoTo(float targetPos) {
  if (targetPos < 0 || targetPos > maxPos) {
    Serial.println("Error: Target position out of bounds.");
    return;
  }
  int steps = int((targetPos - currentPos) / IN_PER_STEP);
  Serial.println("Moving " + String(steps) + " steps.");
  gantryMove(steps);
}

void homeGantry() {
  // Move in the negative direction until the limit switch is pressed
  digitalWrite(DIR, LOW);
  while (digitalRead(LIMIT_SWITCH_1) == HIGH) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
  }
  delayMicroseconds(100);

  // Move back until the limit switch is no longer pressed
  digitalWrite(DIR, HIGH);
  while (digitalRead(LIMIT_SWITCH_1) == LOW) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
  }
  delayMicroseconds(100);

  currentPos = 0;

  // Move in the positive direction until the limit switch is pressed
  digitalWrite(DIR, HIGH);
  while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
    currentPos += IN_PER_STEP;
  }
  delayMicroseconds(100);
  
  // Move back until the limit switch is no longer pressed
  digitalWrite(DIR, LOW);
  while (digitalRead(LIMIT_SWITCH_2) == LOW) {
    digitalWrite(STP, HIGH);
    delayMicroseconds(SLOW_SPEED);
    digitalWrite(STP, LOW);
    delayMicroseconds(SLOW_SPEED);
    currentPos -= IN_PER_STEP;
  }
  delayMicroseconds(100);

  maxPos = currentPos;
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
  homeGantry();
  Serial.println("Homing complete. Max position: " + String(maxPos) + " inches. Current position: " + String(currentPos) + " inches.");
  Serial.println("Ready to receive target position commands.");
}

void loop() {
  String incomingStr = Serial.readStringUntil('\n');
  if (incomingStr == "") {
    return;
  }
  float targetPos = incomingStr.toFloat();
  Serial.println("Moving to position: " + String(targetPos) + " inches.");
  gantryGoTo(targetPos);
  Serial.println("Moved to position: " + String(currentPos) + " inches.");
}