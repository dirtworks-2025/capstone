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
#define RIGHT_FORWARD_PWM 3
#define RIGHT_BACKWARD_PWM 2
#define RIGHT_FORWARD_EN 4
#define RIGHT_BACKWARD_EN 5
#define LEFT_FORWARD_PWM 6
#define LEFT_BACKWARD_PWM 7
#define LEFT_FORWARD_EN 8
#define LEFT_BACKWARD_EN 9

// Stirrup hoe forward / backward pins
#define HOE_ENCODER_CLK 18
#define HOE_ENCODER_DT 19

#define HOE_FORWARD_PWM 10
#define HOE_BACKWARD_PWM 11
#define HOE_FORWARD_EN 12
#define HOE_BACKWARD_EN 13

#define HOE_LIMIT_SWITCH 44

// Hoe speeds and variables
#define HOE_UP_SPEED 30
#define HOE_DOWN_SPEED 15

#define HOE_HOME_POSITION -80 // Encoder counts
bool hoeHomed = false; // True if the hoe has been homed

volatile int currentHoePosition = 0; // Measured in encoder counts

volatile int lastHoeEncoderCLK = 0;
volatile int lastHoeEncoderDT = 0;

// Gantry position and limits
const float IN_PER_STEP = 0.025; // Estimated distance per step
float currentPos = 0;            // Inches
float minPos = 0;                // Inches - Soft limit - Will be set to 2 inches from lower limit switch after homing
float maxPos = 0;                // Inches - Soft limit - Will be set to 2 inches from upper limit switch after homing
bool gantryHomed = false;

// Standard delays
#define GANTRY_HOMING_STEP_DELAY_MS 10
#define HOE_HOMING_LOOP_DELAY_MS 10
#define AWAIT_NEXT_CMD_MS 10
#define TANK_DRIVE_ACCEL_DELAY_MS 10 // Delay between tank drive speed changes

// Current move commands (these will be set by the command interpreter, then eexecuted in the task queue)
int cmdRightTankDriveSpeed = 0;
int cmdLeftTankDriveSpeed = 0;
float gantryStepDelayMs = 0; // Delay between steps in milliseconds (0 = stopped, positive = forward, negative = reverse)
int targetHoePosition = 0;

// Current tank drive speeds (these will lag the commands slightly so the motors can accelerate / decelerate)
int currentRightTankDriveSpeed = 0;
int currentLeftTankDriveSpeed = 0;

#define TANK_DRIVE_ACCEL_STEP 10 // The max amount to increase / decrease the speed by while accelerating / decelerating

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

// Error handling
String prevErrorMsg;

// Logging
bool debug = true; // Set to false to disable debug messages

void throwError(String errorMsg)
{
    if (errorMsg == prevErrorMsg)
    {
        return; // Don't print the same error message twice in a row
    }
    Serial.println("Error: " + errorMsg);
    prevErrorMsg = errorMsg;
}

void maybeLog(String msg)
{
    if (debug)
    {
        Serial.println(msg);
    }
}

bool isGantryAtLimit(float nextPos)
{
    if (digitalRead(LIMIT_SWITCH_1) == LOW || digitalRead(LIMIT_SWITCH_2) == LOW)
    {
        throwError("Limit switch pressed.");
        return true;
    }
    if (nextPos < minPos || nextPos > maxPos)
    {
        throwError("Soft limits exceeded.");
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

void stepReverseUntilLimitSwitch()
{
    gantryHomeStepReverse();
    if (digitalRead(LIMIT_SWITCH_1) == HIGH)
    {
        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepReverseUntilLimitSwitch);
    }
    else
    {
        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepForwardUntilNoLimitSwitch);
    }
}

void stepForwardUntilNoLimitSwitch()
{
    gantryHomeStepForward();
    if (digitalRead(LIMIT_SWITCH_1) == LOW)
    {
        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepForwardUntilNoLimitSwitch);
    }
    else
    {
        currentPos = 0;

        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepForwardUntilLimitSwitch);
    }
}

void stepForwardUntilLimitSwitch()
{
    gantryHomeStepForward();
    if (digitalRead(LIMIT_SWITCH_2) == HIGH)
    {
        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepForwardUntilLimitSwitch);
    }
    else
    {
        // Set 2 inch soft limits
        maxPos = currentPos - 2;
        minPos = 2;

        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepReverseUntilSoftLimits);
    }
}

void stepReverseUntilSoftLimits()
{
    gantryHomeStepReverse();
    if (currentPos > maxPos)
    {
        insertTask(GANTRY_HOMING_STEP_DELAY_MS, stepReverseUntilSoftLimits);
    }
    else
    {
        gantryHomed = true;

        // Add gantry and hoe move tasks to the queue, now that homing is complete
        insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
        insertTask(AWAIT_NEXT_CMD_MS, maybeMoveHoe);

        maybeLog("Homing complete. Max position: " + String(maxPos) + " inches. Current position: " + String(currentPos) + " inches.");
    }
}

void homeGantry()
{
    maybeLog("Homing gantry...");
    insertTask(1000, stepReverseUntilLimitSwitch);
}

void raiseHoeUntilLimitSwitch()
{
    // Move the hoe toward the limit switch until it is pressed
    if (digitalRead(HOE_LIMIT_SWITCH) == HIGH)
    {
        analogWrite(HOE_FORWARD_PWM, HOE_UP_SPEED);
        analogWrite(HOE_BACKWARD_PWM, 0);
        insertTask(HOE_HOMING_LOOP_DELAY_MS, raiseHoeUntilLimitSwitch);
    }
    else
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, 0);
        insertTask(HOE_HOMING_LOOP_DELAY_MS, lowerHoeUntilNoLimitSwitch);
    }
}

void lowerHoeUntilNoLimitSwitch()
{
    // Move the hoe down until the limit switch is not pressed
    if (digitalRead(HOE_LIMIT_SWITCH) == LOW)
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, HOE_DOWN_SPEED);
        insertTask(HOE_HOMING_LOOP_DELAY_MS, lowerHoeUntilNoLimitSwitch);
    }
    else
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, 0);

        hoeHomed = true;
        currentHoePosition = HOE_HOME_POSITION;
        targetHoePosition = HOE_HOME_POSITION + 10; // Just outside soft limits

        maybeLog("Hoe homing complete. Current position: " + String(currentHoePosition) + " counts.");
        maybeLog("Moving hoe to stow position: " + String(targetHoePosition) + " counts.");

        homeGantry(); // Start homing the gantry
    }
}

// This is not designed to be homed asychronously because I don't want the
// robot driving until we know where the hoe is
void homeHoeThenGantry()
{
    maybeLog("Homing hoe...");
    insertTask(0, raiseHoeUntilLimitSwitch);
}

// If the gantry is enabled, take a step in the given direction
// and schedule the next step based on the given delay
void maybeMoveGantry()
{
    if (gantryStepDelayMs == 0 || !gantryHomed)
    {
        insertTask(AWAIT_NEXT_CMD_MS, maybeMoveGantry);
    }
    else
    {
        gantryStep(gantryStepDelayMs > 0);
        insertTask(abs(gantryStepDelayMs), maybeMoveGantry);
    }
}

void dontMoveHoe()
{
    // Stop the hoe from moving, but check again later
    analogWrite(HOE_FORWARD_PWM, 0);
    analogWrite(HOE_BACKWARD_PWM, 0);
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveHoe);
}

void maybeMoveHoe()
{
    if (!hoeHomed) 
    {
        throwError("Hoe not homed. Cannot move hoe.");
        dontMoveHoe();
        return;
    }
    // Verify the target position is within the limits of the hoe
    const int hoeSoftLimitMargin = 5;
    if (targetHoePosition < HOE_HOME_POSITION + hoeSoftLimitMargin)
    {
        throwError("Target hoe position out of range: " + String(targetHoePosition) + " counts.");
        dontMoveHoe();
        return;
    }
    if (targetHoePosition > hoeSoftLimitMargin)
    {
        throwError("Target hoe position out of range: " + String(targetHoePosition) + " counts.");
        dontMoveHoe();
        return;
    }

    int delta = targetHoePosition - currentHoePosition;

    // Deadzone
    if (abs(delta) < 2)
    {
        dontMoveHoe();
        return;
    }

    if (delta < 0)
    {
        analogWrite(HOE_FORWARD_PWM, HOE_UP_SPEED);
        analogWrite(HOE_BACKWARD_PWM, 0);
    }
    else if (delta > 0)
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, HOE_DOWN_SPEED);
    }
    else
    {
        analogWrite(HOE_FORWARD_PWM, 0);
        analogWrite(HOE_BACKWARD_PWM, 0);
    }

    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveHoe);
}

void maybeMoveTankDrive()
{
    // Smoothly accelerate / decelerate the tank drive speeds
    int rightAccelStep = min(TANK_DRIVE_ACCEL_STEP, abs(cmdRightTankDriveSpeed - currentRightTankDriveSpeed));
    int leftAccelStep = min(TANK_DRIVE_ACCEL_STEP, abs(cmdLeftTankDriveSpeed - currentLeftTankDriveSpeed));

    if (cmdRightTankDriveSpeed > currentRightTankDriveSpeed)
    {
        currentRightTankDriveSpeed += rightAccelStep;
    }
    else if (cmdRightTankDriveSpeed < currentRightTankDriveSpeed)
    {
        currentRightTankDriveSpeed -= rightAccelStep;
    }
    if (cmdLeftTankDriveSpeed > currentLeftTankDriveSpeed)
    {
        currentLeftTankDriveSpeed += leftAccelStep;
    }
    else if (cmdLeftTankDriveSpeed < currentLeftTankDriveSpeed)
    {
        currentLeftTankDriveSpeed -= leftAccelStep;
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
    pinMode(HOE_ENCODER_CLK, INPUT_PULLUP);
    pinMode(HOE_ENCODER_DT, INPUT_PULLUP);

    pinMode(HOE_FORWARD_PWM, OUTPUT);
    pinMode(HOE_BACKWARD_PWM, OUTPUT);
    pinMode(HOE_FORWARD_EN, OUTPUT);
    pinMode(HOE_BACKWARD_EN, OUTPUT);

    pinMode(HOE_LIMIT_SWITCH, INPUT_PULLUP);

    digitalWrite(HOE_FORWARD_PWM, LOW);
    digitalWrite(HOE_BACKWARD_PWM, LOW);
    digitalWrite(HOE_FORWARD_EN, HIGH);
    digitalWrite(HOE_BACKWARD_EN, HIGH);

    attachInterrupt(digitalPinToInterrupt(HOE_ENCODER_CLK), updateHoeEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HOE_ENCODER_DT), updateHoeEncoder, CHANGE);

    lastHoeEncoderCLK = digitalRead(HOE_ENCODER_CLK);
    lastHoeEncoderDT = digitalRead(HOE_ENCODER_DT);
}

void initializeRadio()
{
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    maybeLog("Radio initialized.");
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    initializeRadio();
    initializeHoe();
    initializeGantry();
    initializeTankDrive();

    // Home the gantry and hoe
    homeHoeThenGantry();

    // Schedule the first move tasks
    // These tasks will infinitely reschedule themselves until the program is terminated
    // Note: the gantry and hoe tasks will be scheduled to run after homing is complete
    insertTask(AWAIT_NEXT_CMD_MS, maybeMoveTankDrive);
}

// Command types described in comment in rc_controller.ino
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

    // TODO: better data validation
    if (tokens[0] == "drive")
    {
        cmdRightTankDriveSpeed = tokens[1].toInt();
        cmdLeftTankDriveSpeed = tokens[2].toInt();
    }
    else if (tokens[0] == "gantry")
    {
        gantryStepDelayMs = tokens[1].toInt() / 1000; // Convert microseconds to milliseconds
    }
    else if (tokens[0] == "hoe")
    {
        targetHoePosition = tokens[1].toInt();
    }
    else if (tokens[0] == "mode")
    {
        handleModeChange(tokens[1].toInt(), cmd);
    }
    else
    {
        throwError("Unrecognized command: " + cmd);
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

void broadcastToRaspi(String msg)
{
    Serial.println(msg);
}

void checkForRcCmd()
{
    if (radio.available())
    {
        char text[32] = "";
        radio.read(&text, sizeof(text));
        lastCmdReceived = millis();
        interpretCmd(String(text));
    }
    else if (millis() - lastCmdReceived > CMD_TIMEOUT_MS)
    {
        // If no command has been received for the timeout interval, stop the robot
        // and set the control mode to STOP
        // The robot will resume when it begins receiving commands again
        if (controlMode != MODE_STOP)
        {
            maybeLog("No command received for over " + String(CMD_TIMEOUT_MS) + " ms.");
            controlMode = MODE_STOP;
            handleStop();
        }
    }
}

void handleModeChange(byte newMode, String cmd)
{
    if (newMode == controlMode)
    {
        return; // No change in mode
    }
    controlMode = newMode;
    maybeLog("Control mode changed to " + String(newMode));

    broadcastToRaspi(cmd);

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
        throwError("Invalid mode: " + String(newMode));
    }
}

void handleStop()
{
    digitalWrite(GANTRY_EN, HIGH);
    digitalWrite(RIGHT_FORWARD_EN, LOW);
    digitalWrite(RIGHT_BACKWARD_EN, LOW);
    digitalWrite(LEFT_FORWARD_EN, LOW);
    digitalWrite(LEFT_BACKWARD_EN, LOW);
    digitalWrite(HOE_FORWARD_EN, LOW);
    digitalWrite(HOE_BACKWARD_EN, LOW);

    cmdRightTankDriveSpeed = 0;
    cmdLeftTankDriveSpeed = 0;
    currentRightTankDriveSpeed = 0;
    currentLeftTankDriveSpeed = 0;
    gantryStepDelayMs = 0;
    targetHoePosition = currentHoePosition;

    maybeLog("Robot stopped.");
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
    targetHoePosition = currentHoePosition;

    maybeLog("Robot resumed.");
}

void loop()
{
    checkForRcCmd();
    maybeCheckForRaspiCmd();
    runAllReadyTasks();
}

void updateHoeEncoder()
{
    int currentCLK = digitalRead(HOE_ENCODER_CLK);
    int currentDT = digitalRead(HOE_ENCODER_DT);

    if (currentCLK != lastHoeEncoderCLK)
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
    else if (currentDT != lastHoeEncoderDT)
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

    lastHoeEncoderCLK = currentCLK;
    lastHoeEncoderDT = currentDT;
}