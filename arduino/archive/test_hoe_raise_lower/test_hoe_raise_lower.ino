#include <Arduino.h>

#define CLK 18 // Clock pin / Hall Sensor A / Green wire
#define DT 19  // Data pin / Hall Sensor B / Yellow wire

#define MOTOR_FORWARD_PWM 11
#define MOTOR_BACKWARD_PWM 10
#define MOTOR_FORWARD_EN 12
#define MOTOR_BACKWARD_EN 13
#define HOE_LIMIT_SWITCH 44

#define HOE_RAISE_SPEED 40 // Requires more power to raise the hoe
#define HOE_LOWER_SPEED 10 // Requires less power to lower the hoe

volatile int currentHoePosition = 0; // Current encoder value
int targetHoePivotValue = 0;           // Target encoder value

volatile int lastCLK = digitalRead(CLK);
volatile int lastDT = digitalRead(DT);

void setup()
{

    Serial.begin(9600);
    while (!Serial)
        ; // Wait for serial to initialize

    // put your setup code here, to run once:
    pinMode(CLK, INPUT_PULLUP);
    pinMode(DT, INPUT_PULLUP);
    pinMode(MOTOR_FORWARD_PWM, OUTPUT);
    pinMode(MOTOR_BACKWARD_PWM, OUTPUT);
    pinMode(MOTOR_FORWARD_EN, OUTPUT);
    pinMode(MOTOR_BACKWARD_EN, OUTPUT);
    pinMode(HOE_LIMIT_SWITCH, INPUT_PULLUP);

    analogWrite(MOTOR_FORWARD_PWM, 0);
    analogWrite(MOTOR_BACKWARD_PWM, 0);
    digitalWrite(MOTOR_FORWARD_EN, HIGH);
    digitalWrite(MOTOR_BACKWARD_EN, HIGH);

    attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DT), updateEncoder, CHANGE);

    homeStirrupHoe();
}

void loop()
{
    lowerHoe();
    stowHoe();

    delay(1000);
}

void maybeMoveHoe()
{
    int delta = targetHoePivotValue - currentHoePosition;
    // Deadzone
    if (abs(delta) < 2)
    {
        analogWrite(MOTOR_FORWARD_PWM, 0);
        analogWrite(MOTOR_BACKWARD_PWM, 0);
        return;
    }

    // Move the hoe motor in the direction of the target value
    if (delta > 0)
    {
        analogWrite(MOTOR_FORWARD_PWM, HOE_LOWER_SPEED);
        analogWrite(MOTOR_BACKWARD_PWM, 0);
    }
    else if (delta < 0)
    {
        analogWrite(MOTOR_FORWARD_PWM, 0);
        analogWrite(MOTOR_BACKWARD_PWM, HOE_RAISE_SPEED);
    }
    else
    {
        analogWrite(MOTOR_FORWARD_PWM, 0);
        analogWrite(MOTOR_BACKWARD_PWM, 0);
    }
}

void homeStirrupHoe()
{
    while (digitalRead(HOE_LIMIT_SWITCH) == HIGH)
    {
        analogWrite(MOTOR_BACKWARD_PWM, HOE_RAISE_SPEED);
        analogWrite(MOTOR_FORWARD_PWM, 0);
    }
    analogWrite(MOTOR_BACKWARD_PWM, 0);

    while (digitalRead(HOE_LIMIT_SWITCH) == LOW)
    {
        analogWrite(MOTOR_FORWARD_PWM, HOE_LOWER_SPEED);
        analogWrite(MOTOR_BACKWARD_PWM, 0);
    }
    analogWrite(MOTOR_FORWARD_PWM, 0);

    currentHoePosition = -70;
    targetHoePivotValue = -70;
}

void lowerHoe()
{
    targetHoePivotValue = 0;
    while (currentHoePosition != targetHoePivotValue)
    {
        maybeMoveHoe();
        delay(10);
    }
}

void stowHoe()
{
    targetHoePivotValue = -70;
    while (currentHoePosition != targetHoePivotValue)
    {
        maybeMoveHoe();
        delay(10);
    }
}

void updateEncoder()
{
    int currentCLK = digitalRead(CLK);
    int currentDT = digitalRead(DT);

    if (currentCLK != lastCLK)
    {
        if (currentDT != currentCLK)
        {
            currentHoePosition++; // CW
        }
        else
        {
            currentHoePosition--; // CCW
        }
    }
    else if (currentDT != lastDT)
    {
        if (currentDT != currentCLK)
        {
            currentHoePosition--; // CW
        }
        else
        {
            currentHoePosition++; // CCW
        }
    }

    lastCLK = currentCLK;
    lastDT = currentDT;
}
