#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define MAX_TASKS 8 // Maximum number of tasks that can be scheduled

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
#define LIMIT_SWITCH_1 47
#define LIMIT_SWITCH_2 46

// Tank drive control pins
// Semantics of RIGHT and LEFT might not be correct, but this correctly interprets RC commands
#define RIGHT_FORWARD_PWM    3
#define RIGHT_BACKWARD_PWM   2
#define RIGHT_FORWARD_EN     4
#define RIGHT_BACKWARD_EN    5
#define LEFT_FORWARD_PWM     6
#define LEFT_BACKWARD_PWM    7 
#define LEFT_FORWARD_EN      8
#define LEFT_BACKWARD_EN     9

// Stirrup hoe forward / backward pins
#define HOE_FORWARD_PWM 11
#define HOE_BACKWARD_PWM 10
#define HOE_FORWARD_EN 12
#define HOE_BACKWARD_EN 13

// Gantry position and limits
const float IN_PER_STEP = 0.025; // Estimated distance per step
float currentPos = 0; // Inches
float minPos = 0;     // Inches - Soft limit - Will be set to 2 inches from lower limit switch after homing
float maxPos = 0;     // Inches - Soft limit - Will be set to 2 inches from upper limit switch after homing
bool gantryHomed = false;

// Standard delays
#define HOMING_STEP_DELAY_MS 10
#define AWAIT_NEXT_CMD_MS 100
#define TANK_DRIVE_ACCEL_DELAY_MS 10 // Delay between tank drive speed changes

// Current move commands (these will be set by the command interpreter, then eexecuted in the task queue)
int cmdRightTankDriveSpeed = 0;
int cmdLeftTankDriveSpeed = 0;
float gantryStepDelayMs = 0; // Delay between steps in milliseconds (0 = stopped, positive = forward, negative = reverse)
int hoeUpDownSpeed = 0;

// Current tank drive speeds (these will lag the commands slightly so the motors can accelerate / decelerate)
int currentRightTankDriveSpeed = 0;
int currentLeftTankDriveSpeed = 0;

// Mode state
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_STOP 2
byte controlMode = MODE_MANUAL;

// Verifying in range of RC controller
unsigned long lastCmdReceived = 0;
const unsigned long CMD_TIMEOUT_MS = 1000; // must receive a command within this interval

// Radio
#define CE_PIN 30
#define CSN_PIN 31
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

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
        Serial.println("Hit first limit switch.");

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

void maybeMoveHoeUpDown()
{
    if (hoeUpDownSpeed > 0)
    {
        analogWrite(HOE_FORWARD_PWM, hoeUpDownSpeed);
        analogWrite(HOE_BACKWARD_PWM, 0);
    }
    else if (hoeUpDownSpeed < 0)
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, -hoeUpDownSpeed);
    }
    else
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, 0);
    }

    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveHoeUpDown);
}

void maybeMoveTankDrive()
{
    // Smoothly accelerate / decelerate the tank drive speeds
    if (cmdRightTankDriveSpeed > currentRightTankDriveSpeed)
    {
        currentRightTankDriveSpeed += 1;
    }
    else if (cmdRightTankDriveSpeed < currentRightTankDriveSpeed)
    {
        currentRightTankDriveSpeed -= 1;
    }
    if (cmdLeftTankDriveSpeed > currentLeftTankDriveSpeed)
    {
        currentLeftTankDriveSpeed += 1;
    }
    else if (cmdLeftTankDriveSpeed < currentLeftTankDriveSpeed)
    {
        currentLeftTankDriveSpeed -= 1;
    }

    // Set the tank drive speeds to the current speeds
    if (currentRightTankDriveSpeed > 0)
    {
        analogWrite(RIGHT_FORWARD_PWM, currentRightTankDriveSpeed);
        analogWrite(RIGHT_BACKWARD_PWM, 0);
    }
    else if (currentRightTankDriveSpeed < 0)
    {
        analogWrite(RIGHT_FORWARD_PWM, 0);
        analogWrite(RIGHT_BACKWARD_PWM, -currentRightTankDriveSpeed);
    }
    else
    {
        analogWrite(RIGHT_FORWARD_PWM, 0);
        analogWrite(RIGHT_BACKWARD_PWM, 0);
    }

    if (currentLeftTankDriveSpeed > 0)
    {
        analogWrite(LEFT_FORWARD_PWM, currentLeftTankDriveSpeed);
        analogWrite(LEFT_BACKWARD_PWM, 0);
    }
    else if (currentLeftTankDriveSpeed < 0)
    {
        analogWrite(LEFT_FORWARD_PWM, 0);
        analogWrite(LEFT_BACKWARD_PWM, -currentLeftTankDriveSpeed);
    }
    else
    {
        analogWrite(LEFT_FORWARD_PWM, 0);
        analogWrite(LEFT_BACKWARD_PWM, 0);
    }

    insertTask(TANK_DRIVE_ACCEL_DELAY_MS, maybeMoveTankDrive);
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
    pinMode(RIGHT_FORWARD_PWM, OUTPUT);
    pinMode(RIGHT_BACKWARD_PWM, OUTPUT);
    pinMode(LEFT_FORWARD_PWM, OUTPUT);
    pinMode(LEFT_BACKWARD_PWM, OUTPUT);

    pinMode(RIGHT_FORWARD_EN, OUTPUT);
    pinMode(RIGHT_BACKWARD_EN, OUTPUT);
    pinMode(LEFT_FORWARD_EN, OUTPUT);
    pinMode(LEFT_BACKWARD_EN, OUTPUT);

    digitalWrite(RIGHT_FORWARD_PWM, LOW);
    digitalWrite(RIGHT_BACKWARD_PWM, LOW);
    digitalWrite(LEFT_FORWARD_PWM, LOW);
    digitalWrite(LEFT_BACKWARD_PWM, LOW);

    digitalWrite(RIGHT_FORWARD_EN, HIGH);
    digitalWrite(RIGHT_BACKWARD_EN, HIGH);
    digitalWrite(LEFT_FORWARD_EN, HIGH);
    digitalWrite(LEFT_BACKWARD_EN, HIGH);
}

void initializeHoe()
{
    pinMode(HOE_FORWARD_PWM, OUTPUT);
    pinMode(HOE_BACKWARD_PWM, OUTPUT);
    pinMode(HOE_FORWARD_EN, OUTPUT);
    pinMode(HOE_BACKWARD_EN, OUTPUT);

    digitalWrite(HOE_FORWARD_PWM, LOW);
    digitalWrite(HOE_BACKWARD_PWM, LOW);

    digitalWrite(HOE_FORWARD_EN, HIGH);
    digitalWrite(HOE_BACKWARD_EN, HIGH);
}

void initializeRadio() 
{
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    Serial.println("Radio initialized.");
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    initializeGantry();
    initializeTankDrive();
    initializeHoe();
    initializeRadio();
    // Schedule the first move tasks
    // These tasks will infinitely reschedule themselves until the program is terminated
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveHoeUpDown);
}

// Command types:
// drive <leftSpeed> <rightSpeed>
// hoe <gantryStepDelay_uS> <upDownSpeed>
// mode <number> (0 = auto, 1 = manual, 2 = stop)

// Note: Commands are initially interpretted generically as <token1> <token2> <token3>
void interpretCmd(String cmd)
{
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

    if (tokens[0] == "drive")
    {
        cmdRightTankDriveSpeed = tokens[1].toInt();
        cmdLeftTankDriveSpeed = tokens[2].toInt();
    }
    else if (tokens[0] == "hoe")
    {
        gantryStepDelayMs = tokens[1].toInt() / 1000; // Convert microseconds to milliseconds
        hoeUpDownSpeed = tokens[2].toInt();
    }
    else if (tokens[0] == "mode")
    {
        handleModeChange(tokens[1].toInt());
    }
    else
    {
        Serial.println("Error: Unrecognized command.");
    }
}

void maybeCheckForRaspiCmd()
{
    if (controlMode != MODE_AUTO)
    {
        return;
    }
    if (Serial.available() > 0)
    {
        interpretCmd(Serial.readStringUntil('\n'));
    }
}

void checkForRcCmd()
{
    if (radio.available())
    {
        char text[32] = "";
        radio.read(&text, sizeof(text));
        lastCmdReceived = millis();
        // Serial.print("Received command: " + String(text) + "\n");
        interpretCmd(String(text));
    }
    else if (millis() - lastCmdReceived > CMD_TIMEOUT_MS)
    {
        // If no command has been received for the timeout interval, stop the robot
        // and set the control mode to STOP
        // The robot will resume when it begins receiving commands again
        if (controlMode != MODE_STOP)
        {
            Serial.println("No command received for over " + String(CMD_TIMEOUT_MS) + " ms.");
            controlMode = MODE_STOP;
            handleStop();
        }
    }
}

void handleModeChange(byte newMode)
{
    if (newMode == controlMode)
    {
        return; // No change in mode
    }
    controlMode = newMode;
    Serial.println("Control mode changed to " + String(newMode));
    if (newMode == MODE_AUTO)
    {

        handleResume();
    }
    else if (newMode == MODE_MANUAL)
    {
        handleResume();
    }
    else if (newMode == MODE_STOP)
    {
        handleStop();
    }
    else
    {
        controlMode = MODE_STOP;
        handleStop();
        Serial.println("Error: Invalid mode.");
    }
}

void handleStop()
{
    digitalWrite(GANTRY_EN, HIGH);
    digitalWrite(RIGHT_FORWARD_EN, LOW);
    digitalWrite(RIGHT_BACKWARD_EN, LOW);
    digitalWrite(LEFT_FORWARD_EN, LOW);
    digitalWrite(LEFT_BACKWARD_EN, LOW);

    cmdRightTankDriveSpeed = 0;
    cmdLeftTankDriveSpeed = 0;
    currentRightTankDriveSpeed = 0;
    currentLeftTankDriveSpeed = 0;
    gantryStepDelayMs = 0;
    hoeUpDownSpeed = 0;

    Serial.println("Robot stopped.");
}

void handleResume()
{
    digitalWrite(GANTRY_EN, LOW);
    digitalWrite(RIGHT_FORWARD_EN, HIGH);
    digitalWrite(RIGHT_BACKWARD_EN, HIGH);
    digitalWrite(LEFT_FORWARD_EN, HIGH);
    digitalWrite(LEFT_BACKWARD_EN, HIGH);

    cmdRightTankDriveSpeed = 0;
    cmdLeftTankDriveSpeed = 0;
    currentRightTankDriveSpeed = 0;
    currentLeftTankDriveSpeed = 0;
    gantryStepDelayMs = 0;
    hoeUpDownSpeed = 0;

    Serial.println("Robot resumed.");
}

void loop()
{
    checkForRcCmd();
    maybeCheckForRaspiCmd();
    runAllReadyTasks();
}