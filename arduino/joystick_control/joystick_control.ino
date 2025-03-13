#include <Arduino.h>

// Joystick pins
#define GANTRY_JOYSTICK_Y A0
#define GANTRY_JOYSTICK_X A1
#define TANK_JOYSTICK_Y A2
#define TANK_JOYSTICK_X A3

// Stepper motor control pins
#define GANTRY_EN 22
#define GANTRY_DIR 23
#define GANTRY_STP 24
#define LIMIT_SWITCH_1 28
#define LIMIT_SWITCH_2 29

// Tank drive control pins
#define RIGHT_FORWARD 5
#define RIGHT_BACKWARD 4
#define LEFT_FORWARD 2
#define LEFT_BACKWARD 3

// Stepper motor constants
#define SLOW_SPEED 5000
#define FAST_SPEED 2000

const float IN_PER_STEP = 0.025; // Estimated distance per step (TODO: measure this and update)

float currentPos = 0; // Inches
float minPos = 0;     // Inches
float maxPos = 0;     // Inches

// Move the stepper motor a certain number of steps (positive or negative)
void gantryMove(int steps, int speed = FAST_SPEED)
{
    digitalWrite(GANTRY_DIR, (steps > 0) ? HIGH : LOW);
    for (int i = 0; i < abs(steps); i++)
    {
        float nextPos = IN_PER_STEP * (steps > 0 ? 1 : -1);
        // Check for limit switch press
        if (digitalRead(LIMIT_SWITCH_1) == LOW || digitalRead(LIMIT_SWITCH_2) == LOW)
        {
            Serial.println("Error: Limit switch pressed.");
            return;
        }
        // Check for soft limits
        if (nextPos < minPos || nextPos > maxPos)
        {
            Serial.println("Error: Soft limits exceeded.");
            return;
        }

        // Move the stepper motor
        digitalWrite(GANTRY_STP, HIGH);
        delayMicroseconds(speed);
        digitalWrite(GANTRY_STP, LOW);
        delayMicroseconds(speed);

        // Only update the position if the move was actually executed
        currentPos = nextPos;
    }
}

void gantryGoTo(float targetPos)
{
    if (targetPos < 0 || targetPos > maxPos)
    {
        Serial.println("Error: Target position out of bounds.");
        return;
    }
    int steps = int((targetPos - currentPos) / IN_PER_STEP);
    gantryMove(steps);
    delayMicroseconds(100);
}

void homeGantry()
{
    // Move in the negative direction until the limit switch is pressed
    digitalWrite(GANTRY_DIR, LOW);
    while (digitalRead(LIMIT_SWITCH_1) == HIGH)
    {
        digitalWrite(GANTRY_STP, HIGH);
        delayMicroseconds(SLOW_SPEED);
        digitalWrite(GANTRY_STP, LOW);
        delayMicroseconds(SLOW_SPEED);
    }
    delayMicroseconds(100);

    // Move back until the limit switch is no longer pressed
    digitalWrite(GANTRY_DIR, HIGH);
    while (digitalRead(LIMIT_SWITCH_1) == LOW)
    {
        digitalWrite(GANTRY_STP, HIGH);
        delayMicroseconds(SLOW_SPEED);
        digitalWrite(GANTRY_STP, LOW);
        delayMicroseconds(SLOW_SPEED);
    }
    delayMicroseconds(100);

    currentPos = 0;

    // Move in the positive direction until the limit switch is pressed
    digitalWrite(GANTRY_DIR, HIGH);
    while (digitalRead(LIMIT_SWITCH_2) == HIGH)
    {
        digitalWrite(GANTRY_STP, HIGH);
        delayMicroseconds(SLOW_SPEED);
        digitalWrite(GANTRY_STP, LOW);
        delayMicroseconds(SLOW_SPEED);
        currentPos += IN_PER_STEP;
    }
    delayMicroseconds(100);

    // Set 2 inch soft limits
    maxPos = currentPos - 2;
    minPos = 2;

    // Move back until outside the soft limits
    digitalWrite(GANTRY_DIR, LOW);
    while (currentPos > maxPos)
    {
        digitalWrite(GANTRY_STP, HIGH);
        delayMicroseconds(SLOW_SPEED);
        digitalWrite(GANTRY_STP, LOW);
        delayMicroseconds(SLOW_SPEED);
        currentPos -= IN_PER_STEP;
    }
    delayMicroseconds(100);
}

void maybeMoveGantry()
{
    // Continuously read the joystick and move the gantry while the joystick is outside the deadzone
    // This function is blocking when the joystick is outside the deadzone
    // TODO: add threads to make this non-blocking to enable gantry movement while driving
    while (true) {
        int x = analogRead(GANTRY_JOYSTICK_X);
        int xNormalized = map(x, 0, 1023, -100, 100);

        // Set deadzone
        if (abs(xNormalized) < 10)
        {
            return;
        }

        int stepDelay = map(abs(xNormalized), 0, 100, 5000, 1500);
        Serial.println("Step delay: " + String(stepDelay));

        // Move the gantry
        int stepsPerLoop = 5;
        gantryMove(xNormalized > 0 ? stepsPerLoop : -stepsPerLoop, stepDelay);
    }
}

void maybeMoveTankDrive()
{
    // Continuously read the joystick and move the tank drive while the joystick is outside the deadzone
    // This function is blocking when the joystick is outside the deadzone

    while (true) {
        int x = analogRead(TANK_JOYSTICK_X);
        int y = analogRead(TANK_JOYSTICK_Y);
        float xNormalized = map(x, 0, 1023, -100, 100) / 100.0;
        float yNormalized = map(y, 0, 1023, -100, 100) / 100.0;

        // Set deadzone per axis
        if (abs(xNormalized) < 0.1)
        {
            xNormalized = 0.0;
        }
        if (abs(yNormalized) < 0.1)
        {
            yNormalized = 0.0;
        }

        // Escape if both axes are in the deadzone
        if (xNormalized == 0.0 && yNormalized == 0.0)
        {
            analogWrite(RIGHT_FORWARD, 0);
            analogWrite(RIGHT_BACKWARD, 0);
            analogWrite(LEFT_FORWARD, 0);
            analogWrite(LEFT_BACKWARD, 0);
            return;
        }

        // Get speed values
        // Funny enough, the transformation from (x,y) to (L,R) is a 45 degree rotation
        float speedLimit = 0.4;
        float leftSpeed = (yNormalized + xNormalized) * (255 / 2) * speedLimit;
        float rightSpeed = (yNormalized - xNormalized) * (255 / 2) * speedLimit;

        // Move the tank drive
        if (rightSpeed > 0.0)
        {
            analogWrite(RIGHT_FORWARD, rightSpeed);
            analogWrite(RIGHT_BACKWARD, 0);
        }
        else if (rightSpeed < 0.0)
        {
            analogWrite(RIGHT_BACKWARD, -rightSpeed);
            analogWrite(RIGHT_FORWARD, 0);
        }
        else
        {
            analogWrite(RIGHT_FORWARD, 0);
            analogWrite(RIGHT_BACKWARD, 0);
        }

        if (leftSpeed > 0.0)
        {
            analogWrite(LEFT_FORWARD, leftSpeed);
            analogWrite(LEFT_BACKWARD, 0);
        }
        else if (leftSpeed < 0.0)
        {
            analogWrite(LEFT_BACKWARD, -leftSpeed);
            analogWrite(LEFT_FORWARD, 0);
        }
        else
        {
            analogWrite(LEFT_FORWARD, 0);
            analogWrite(LEFT_BACKWARD, 0);
        }
    }
}

void initializeGantry()
{
    pinMode(GANTRY_DIR, OUTPUT);
    pinMode(GANTRY_STP, OUTPUT);
    pinMode(GANTRY_EN, OUTPUT);
    pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
    pinMode(GANTRY_JOYSTICK_X, INPUT);
    pinMode(GANTRY_JOYSTICK_Y, INPUT);

    digitalWrite(GANTRY_EN, LOW);
    
    // Home the gantry
    Serial.println("Homing gantry...");
    homeGantry();
    Serial.println("Homing complete. Max position: " + String(maxPos) + " inches. Current position: " + String(currentPos) + " inches.");
}

void initializeTankDrive()
{
    pinMode(RIGHT_FORWARD, OUTPUT);
    pinMode(RIGHT_BACKWARD, OUTPUT);
    pinMode(LEFT_FORWARD, OUTPUT);
    pinMode(LEFT_BACKWARD, OUTPUT);

    digitalWrite(RIGHT_FORWARD, LOW);
    digitalWrite(RIGHT_BACKWARD, LOW);
    digitalWrite(LEFT_FORWARD, LOW);
    digitalWrite(LEFT_BACKWARD, LOW);

    pinMode(TANK_JOYSTICK_X, INPUT);
    pinMode(TANK_JOYSTICK_Y, INPUT);
    
    Serial.println("Tank drive initialized.");
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    initializeTankDrive();
    initializeGantry();
}

void loop()
{
    maybeMoveTankDrive();
    maybeMoveGantry();
    delay(100);
}