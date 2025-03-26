#include <Arduino.h>

#define MAX_TASKS 16

struct Task
{
    unsigned long scheduledTime;
    void (*callback)();
    bool inUse; // false if the slot has not been allocated or if the task has been executed
};

Task tasks[MAX_TASKS];

bool insertTask(unsigned long delayMs, void (*callback)())
{
    for (uint8_t i = 0; i < MAX_TASKS; i++)
    {
        if (!tasks[i].inUse)
        {
            tasks[i].scheduledTime = millis() + delayMs;
            tasks[i].callback = callback;
            tasks[i].inUse = true;
            return true;
        }
    }
    return false;
}

// Run up to MAX_TASKS ready tasks
// This function is blocking, but has a limited number of iterations to prevent infinite loops
void runReadyTasks()
{
    for (uint8_t i = 0; i < MAX_TASKS; i++)
    {
        if (tasks[i].inUse && tasks[i].scheduledTime <= millis())
        {
            tasks[i].callback();
            tasks[i].inUse = false;
        }
    }
}

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

// Gantry position and limits
const float IN_PER_STEP = 0.025; // Estimated distance per step
float currentPos = 0; // Inches
float minPos = 0;     // Inches
float maxPos = 0;     // Inches
bool gantryHomed = false;

// Standard delays
#define HOMING_STEP_DELAY_MS 10
#define AWAIT_NEXT_CMD_MS 100

// Current move commands
float gantryStepDelayMs = 0; // Delay between steps in milliseconds (0 = stopped, positive = forward, negative = reverse)
int rightTankDriveSpeed = 0;
int leftTankDriveSpeed = 0;


bool isGantryAtLimit(float nextPos)
{
    if (digitalRead(LIMIT_SWITCH_1) == LOW || digitalRead(LIMIT_SWITCH_2) == LOW)
    {
        Serial.println("Error: Limit switch pressed.");
        return true;
    }
    if (nextPos < minPos || nextPos > maxPos)
    {
        Serial.println("Error: Soft limits exceeded.");
        return true;
    }
    return false;
}

void gantryStep(bool dir, bool ignoreLimits = false)
{
    float nextPos = currentPos + IN_PER_STEP * (dir ? 1 : -1);
    if (!ignoreLimits && isGantryAtLimit(nextPos))
    {
        return;
    }

    digitalWrite(GANTRY_DIR, dir ? HIGH : LOW);
    digitalWrite(GANTRY_STP, HIGH);
    delayMicroseconds(100);
    digitalWrite(GANTRY_STP, LOW);

    currentPos = nextPos;
}

// Wrapper functions for moving the gantry while ignoring limits
void gantryHomeStepReverse()
{
    gantryStep(false, true);
}

void gantryHomeStepForward()
{
    gantryStep(true, true);
}

void stepReverseUntilLimitSwitch() {
    gantryHomeStepReverse();
    if (digitalRead(LIMIT_SWITCH_1) == HIGH) {
        insertTask(HOMING_STEP_DELAY_MS, stepReverseUntilLimitSwitch);
    } else {
        insertTask(HOMING_STEP_DELAY_MS, stepForwardUntilNoLimitSwitch);
    }
}

void stepForwardUntilNoLimitSwitch() {
    gantryHomeStepForward();
    if (digitalRead(LIMIT_SWITCH_1) == LOW) {
        insertTask(HOMING_STEP_DELAY_MS, stepForwardUntilNoLimitSwitch);
    } else {
        currentPos = 0;

        insertTask(HOMING_STEP_DELAY_MS, stepForwardUntilLimitSwitch);
    }
}

void stepForwardUntilLimitSwitch() {
    gantryHomeStepForward();
    if (digitalRead(LIMIT_SWITCH_2) == HIGH) {
        insertTask(HOMING_STEP_DELAY_MS, stepForwardUntilLimitSwitch);
    } else {
        // Set 2 inch soft limits
        maxPos = currentPos - 2;
        minPos = 2;

        insertTask(HOMING_STEP_DELAY_MS, stepReverseUntilSoftLimits);
    }
}

void stepReverseUntilSoftLimits() {
    gantryHomeStepReverse();
    if (currentPos > maxPos) {
        insertTask(HOMING_STEP_DELAY_MS, stepReverseUntilSoftLimits);
    } else {
        gantryHomed = true;
        Serial.println("Homing complete. Max position: " + String(maxPos) + " inches. Current position: " + String(currentPos) + " inches.");
    }
}

void homeGantry()
{
    Serial.println("Homing gantry...");
    insertTask(0, stepReverseUntilLimitSwitch);
}

// If the gantry is enabled, take a step in the given direction
// and schedule the next step based on the given delay
void maybeMoveGantry()
{
    if (gantryStepDelayMs == 0 || !gantryHomed) {
        insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
    } else {
        gantryStep(gantryStepDelayMs > 0);
        insertTask(abs(gantryStepDelayMs), maybeMoveGantry);
    }
}

void maybeMoveTankDrive()
{
    if (rightTankDriveSpeed > 0)
    {
        analogWrite(RIGHT_FORWARD, rightTankDriveSpeed);
        analogWrite(RIGHT_BACKWARD, 0);
    }
    else if (rightTankDriveSpeed < 0)
    {
        analogWrite(RIGHT_FORWARD, 0);
        analogWrite(RIGHT_BACKWARD, -rightTankDriveSpeed);
    }
    else
    {
        analogWrite(RIGHT_FORWARD, 0);
        analogWrite(RIGHT_BACKWARD, 0);
    }

    if (leftTankDriveSpeed > 0)
    {
        analogWrite(LEFT_FORWARD, leftTankDriveSpeed);
        analogWrite(LEFT_BACKWARD, 0);
    }
    else if (leftTankDriveSpeed < 0)
    {
        analogWrite(LEFT_FORWARD, 0);
        analogWrite(LEFT_BACKWARD, -leftTankDriveSpeed);
    }
    else
    {
        analogWrite(LEFT_FORWARD, 0);
        analogWrite(LEFT_BACKWARD, 0);
    }

    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
}

void interpretTankJoystick()
{
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
        rightTankDriveSpeed = 0;
        leftTankDriveSpeed = 0;
        return;
    }

    // Set tank drive speeds
    float speedLimit = 0.6;
    rightTankDriveSpeed = (yNormalized + xNormalized) * (255 / 2) * speedLimit;
    leftTankDriveSpeed = (yNormalized - xNormalized) * (255 / 2) * speedLimit;
}

void interpretGantryJoystick() 
{
    int x = analogRead(GANTRY_JOYSTICK_X);
    int xNormalized = map(x, 0, 1023, -100, 100);

    // Set deadzone
    if (abs(xNormalized) < 10)
    {
        gantryStepDelayMs = 0;
        return;
    }

    float stepDelay = map(abs(xNormalized), 0, 100, 15, 3);
    gantryStepDelayMs = xNormalized > 0 ? stepDelay : -stepDelay;
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
}

void setup()
{
    Serial.begin(9600);
    delay(500);
    initializeGantry();
    initializeTankDrive();
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
}

void loop()
{
    interpretTankJoystick();
    interpretGantryJoystick();
    runReadyTasks();
}