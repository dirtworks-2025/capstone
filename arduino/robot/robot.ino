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
        return false;
    }
}

bool maybeRunOneReadyTask()
{
    for (uint8_t i = 0; i < MAX_TASKS; i++)
    {
        if (tasks[i].inUse && tasks[i].scheduledTime <= millis())
        {
            tasks[i].callback();
            tasks[i].inUse = false;
            return true;
        }
    }
    return false;
}

// Run up to MAX_TASKS ready tasks
// This function is blocking, but has a limited number of iterations to prevent infinite loops
void runAllReadyTasks()
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
int gantryStepDelayMs = 0; // Delay between steps in milliseconds (0 = stopped, positive = forward, negative = reverse)
int rightTankDriveSpeed = 0;
int leftTankDriveSpeed = 0;

void checkGantryLimits(float nextPos)
{
    if (digitalRead(LIMIT_SWITCH_1) == LOW || digitalRead(LIMIT_SWITCH_2) == LOW)
    {
        Serial.println("Error: Limit switch pressed.");
    }
    if (nextPos < minPos || nextPos > maxPos)
    {
        Serial.println("Error: Soft limits exceeded.");
    }
}

void gantryStep(bool dir, bool ignoreLimits = false)
{
    float nextPos = currentPos + IN_PER_STEP * (dir ? 1 : -1);
    if (!ignoreLimits)
    {
        checkGantryLimits(nextPos);
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
        analogWrite(RIGHT_BACKWARD, -rightTankDriveSpeed);
        analogWrite(RIGHT_FORWARD, 0);
    }
    else
    {
        analogWrite(RIGHT_FORWARD, 0);
        analogWrite(RIGHT_BACKWARD, 0);
    }

    if (leftTankDriveSpeed > 0)
    {
        analogWrite(LEFT_BACKWARD, -leftTankDriveSpeed);
        analogWrite(LEFT_FORWARD, 0);
    }
    else if (leftTankDriveSpeed < 0)
    {
        analogWrite(LEFT_FORWARD, leftTankDriveSpeed);
        analogWrite(LEFT_BACKWARD, 0);
    }
    else
    {
        analogWrite(LEFT_FORWARD, 0);
        analogWrite(LEFT_BACKWARD, 0);
    }

    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
}

void initializeGantry()
{
    pinMode(GANTRY_DIR, OUTPUT);
    pinMode(GANTRY_STP, OUTPUT);
    pinMode(GANTRY_EN, OUTPUT);
    pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

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
}

void setup()
{
    Serial.begin(9600);
    delay(500);
    initializeGantry();
    initializeTankDrive();
    // Schedule the first move tasks
    // These tasks will infinitely reschedule themselves until the program is terminated
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
}

// Command types:
// tank <leftSpeed> <rightSpeed>
// gantry <delayMs>
// Note: Commands are first interpretted generically as <token1> <token2> <token3>
void maybeInterpretCmd()
{
    if (Serial.available() > 0)
    {
        String cmd = Serial.readStringUntil('\n');
        String tokens[3];
        int charIdx = 0;
        int tokenIdx = 0;
        while (charIdx < cmd.length())
        {
            if (cmd[charIdx] == ' ')
            {
                tokenIdx++;
            }
            else
            {
                tokens[tokenIdx] += cmd[charIdx];
            }
            charIdx++;
        }

        if (tokens[0] == "tank")
        {
            rightTankDriveSpeed = tokens[1].toInt();
            leftTankDriveSpeed = tokens[2].toInt();
        }
        else if (tokens[0] == "gantry")
        {
            gantryStepDelayMs = tokens[1].toInt();
        }
    }
}

void loop()
{
    maybeInterpretCmd();
    runAllReadyTasks();
}