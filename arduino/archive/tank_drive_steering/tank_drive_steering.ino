#include <Arduino.h>

const int RIGHT_FORWARD = 5;
const int RIGHT_BACKWARD = 4;
const int LEFT_FORWARD = 2;
const int LEFT_BACKWARD = 3;

// Joy stick pins
const int Y_POT = A0;
const int X_POT= A1;

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
    int x_val = analogRead(X_POT);
    int y_val = analogRead(Y_POT);

    // Normalize values
    float x_norm = map(x_val, 0, 1023, -100, 100) / 100.0;
    float y_norm = map(y_val, 0, 1023, -100, 100) / 100.0;

    // Set deadzone
    if (x_norm < 0.1 && x_norm > -0.1)
    {
        x_norm = 0.0;
    }
    if (y_norm < 0.1 && y_norm > -0.1)
    {
        y_norm = 0.0;
    }

    // Serial.println("X: " + String(x_norm) + " Y: " + String(y_norm));
    // delay(100);

    // Get speed values
    float right_speed = (y_norm + x_norm) * (255 / 5);
    float left_speed = (y_norm - x_norm) * (255 / 5);

    // Serial.println("R: " + String(right_speed) + " L: " + String(left_speed));
    // delay(100);

    if (right_speed > 0.0)
    {
        analogWrite(RIGHT_FORWARD, right_speed);
        analogWrite(RIGHT_BACKWARD, 0);
    }
    else if (right_speed < 0.0)
    {
        analogWrite(RIGHT_BACKWARD, -right_speed);
        analogWrite(RIGHT_FORWARD, 0);
    }
    else
    {
        analogWrite(RIGHT_FORWARD, 0);
        analogWrite(RIGHT_BACKWARD, 0);
    }

    if (left_speed > 0.0)
    {
        analogWrite(LEFT_FORWARD, left_speed);
        analogWrite(LEFT_BACKWARD, 0);
    }
    else if (left_speed < 0.0)
    {
        analogWrite(LEFT_BACKWARD, -left_speed);
        analogWrite(LEFT_FORWARD, 0);
    }
    else
    {
        analogWrite(LEFT_FORWARD, 0);
        analogWrite(LEFT_BACKWARD, 0);
    }
}