#include <Arduino.h>

const int RIGHT_FORWARD = 2;
const int RIGHT_BACKWARD = 3;
const int LEFT_FORWARD = 5;
const int LEFT_BACKWARD = 6;

void setup()
{
    pinMode(RIGHT_FORWARD, OUTPUT);
    pinMode(RIGHT_BACKWARD, OUTPUT);
    pinMode(LEFT_FORWARD, OUTPUT);
    pinMode(LEFT_BACKWARD, OUTPUT);

    digitalWrite(RIGHT_FORWARD, LOW);
    digitalWrite(RIGHT_BACKWARD, LOW);
    digitalWrite(LEFT_FORWARD, LOW);
    digitalWrite(LEFT_BACKWARD, LOW);

    delay(1000);
    Serial.begin(9600);
}

void loop()
{
    // Test each motor individually
    for (int i = 0; i < 255; i += 25)
    {
        analogWrite(RIGHT_FORWARD, i);
        delay(100);
    }
    analogWrite(RIGHT_FORWARD, 0);
    delay(1000);

    for (int i = 0; i < 255; i += 25)
    {
        analogWrite(RIGHT_BACKWARD, i);
        delay(100);
    }
    analogWrite(RIGHT_BACKWARD, 0);
    delay(1000);

    for (int i = 0; i < 255; i += 25)
    {
        analogWrite(LEFT_FORWARD, i);
        delay(100);
    }
    analogWrite(LEFT_FORWARD, 0);
    delay(1000);

    for (int i = 0; i < 255; i += 25)
    {
        analogWrite(LEFT_BACKWARD, i);
        delay(100);
    }
    analogWrite(LEFT_BACKWARD, 0);
    delay(1000);
}