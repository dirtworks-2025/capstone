#include <Arduino.h>

#define CLK 18 // Clock pin / Hall Sensor A / Green wire
#define DT 19  // Data pin / Hall Sensor B / Yellow wire

#define MOTOR_FORWARD_PWM 11
#define MOTOR_BACKWARD_PWM 10
#define MOTOR_FORWARD_EN 12
#define MOTOR_BACKWARD_EN 13
#define HOE_LIMIT_SWITCH 44

#define HOE_RAISE_SPEED 30 // Requires more power to raise the hoe
#define HOE_LOWER_SPEED 20 // Requires less power to lower the hoe

volatile int currentHoePosition = 0; // Current encoder value
int targetHoePosition = 0;           // Target encoder value

volatile int lastCLK = digitalRead(CLK);
volatile int lastDT = digitalRead(DT);

void setup()
{

    Serial.begin(9600);
    while (!Serial); // Wait for serial to initialize

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
    delay(1000); // Wait for the hoe to home
}

void loop()
{
    lowerHoe();
    delay(1000);
    raiseHoe();
    delay(1000);
}

void maybeMoveHoe()
{
    int delta = targetHoePosition - currentHoePosition;
    Serial.print("Current: ");
    Serial.print(currentHoePosition);
    Serial.print(" Target: ");
    Serial.print(targetHoePosition);
    Serial.print(" Delta: ");
    Serial.println(delta);
    
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
    targetHoePosition = -70;
    Serial.println("Hoe homed.");
}

void lowerHoe()
{
    Serial.println("Lowering hoe...");
    targetHoePosition = 0;
    while (abs(currentHoePosition - targetHoePosition) > 2)
    {
        maybeMoveHoe();
        delay(10);
    }
    maybeMoveHoe();
}

void raiseHoe()
{
    Serial.println("Raising hoe...");
    targetHoePosition = -70;
    while (abs(currentHoePosition - targetHoePosition) > 2)
    {
        maybeMoveHoe();
        delay(10);
    }
    maybeMoveHoe();
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
