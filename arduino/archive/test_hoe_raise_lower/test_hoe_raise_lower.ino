#include <Arduino.h>

#define ENCODER_A 2
#define ENCODER_B 3

#define NO_SIGNAL  0
#define A_RISING   1
#define A_FALLING  2
#define B_RISING   3
#define B_FALLING  4

#define FORWARD  0
#define BACKWARD 1

int encoderPos = 0;
int lastSignal = NO_SIGNAL;
bool dir = FORWARD;

void setup() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), rise_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), rise_B, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), fall_B, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), fall_A, FALLING);
}

void loop() {

}

void rise_A() {
    if (lastSignal == NO_SIGNAL) {
        lastSignal = A_RISING;
    } else if (lastSignal == A_FALLING) {
        dir = !dir;
    else if (lastSignal == B_RISING) {
        encoderPos--;
    } else if (lastSignal == B_FALLING) {
        encoderPos--;
    }
    lastSignal = A_RISING;
}