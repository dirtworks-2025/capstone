#include <Arduino.h>

#define CLK_PIN 2
#define DT_PIN 3

int encoderPos = 0;

void setup() {
    attachInterrupt(0, updateEncoder, RISING);
}

void loop() {

}

void updateEncoder() {
    if (digitalRead(DT_PIN) == HIGH) {
        encoderPos++;
    } else {
        encoderPos--;
    }
}