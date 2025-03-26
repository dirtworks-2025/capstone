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

void runAllReadyTasks()
{
    while (maybeRunOneReadyTask());
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

// Standard delays
#define HOMING_STEP_DELAY_MS 10
#define 

// Current move commands
int gantryStepDelayMs = 0; // Delay between steps in milliseconds (0 = stopped, positive = forward, negative = reverse)
bool enableGantry = false;

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
        enableGantry = false;
    }
}

void homeGantry()
{
    insertTask(HOMING_STEP_DELAY_MS, stepReverseUntilLimitSwitch);
}

// If the gantry is enabled, take a step in the given direction
// and schedule the next step based on the given delay
void maybeMoveGantry()
{
    if (enableGantry)
    {
        gantryStep(gantryStepDelayMs > 0);
        insertTask(abs(gantryStepDelayMs), maybeMoveGantry);
    } else {
        insertTask(HOMING_STEP_DELAY_MS, stepReverseUntilSoftLimits);
    }
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

void setup()
{
    Serial.begin(9600);
    delay(500);
    initializeGantry();
}