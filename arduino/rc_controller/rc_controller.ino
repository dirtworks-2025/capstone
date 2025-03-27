#include <Arduino.h>

// Joystick pins
#define HOE_JOYSTICK_Y A0
#define HOE_JOYSTICK_X A1
#define DRIVE_JOYSTICK_Y A2
#define DRIVE_JOYSTICK_X A3

// Button pins
#define MODE_SWITCH 2
#define STOP_BUTTON 3

// Mode state
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_STOP 2
byte controlMode = MODE_MANUAL;

// Pulses
unsigned long lastPulseTime = 0;
const unsigned long PULSE_DELAY = 1000;

// Command types:
// drive <leftSpeed> <rightSpeed>
// hoe <gantryStepDelayMs> <upDownSpeed>
// mode <number> (0 = auto, 1 = manual, 2 = stop)
// pulse (robot expects a pulse command every 1 second)

void sendDriveCmd()
{
    int x = analogRead(DRIVE_JOYSTICK_X);
    int y = analogRead(DRIVE_JOYSTICK_Y);
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
        sendCmd("drive 0 0");
        return;
    }

    // Set tank drive speeds
    float speedLimit = 0.6;
    int rightTankDriveSpeed = (yNormalized + xNormalized) * (255 / 2) * speedLimit;
    int leftTankDriveSpeed = (yNormalized - xNormalized) * (255 / 2) * speedLimit;
    sendCmd("drive " + String(leftTankDriveSpeed) + " " + String(rightTankDriveSpeed));
}

void sendHoeCmd()
{
    int x = analogRead(HOE_JOYSTICK_X);
    int y = analogRead(HOE_JOYSTICK_Y);
    int xNormalized = map(x, 0, 1023, -100, 100);
    int yNormalized = map(y, 0, 1023, -100, 100);

    // Set deadzone
    if (abs(xNormalized) < 10)
    {
        xNormalized = 0;
    }
    if (abs(yNormalized) < 10)
    {
        yNormalized = 0;
    }

    // Escape if both axes are in the deadzone
    if (xNormalized == 0 && yNormalized == 0)
    {
        sendCmd("hoe 0 0");
        return;
    }

    // Only listen to the signal with the greatest magnitude
    if (abs(xNormalized) > abs(yNormalized))
    {
        int stepDelay = map(abs(xNormalized), 0, 100, 5000, 1500);
        int gantryStepDelayMs = xNormalized > 0 ? stepDelay : -stepDelay;
        sendCmd("hoe " + String(gantryStepDelayMs) + " 0");
    }
    else
    {
        int upDownSpeed = map(abs(yNormalized), 0, 100, 0, 255);
        int upDownDir = yNormalized > 0 ? 1 : -1;
        sendCmd("hoe 0 " + String(upDownSpeed * upDownDir));
    }
}

void maybeChangeMode() 
{
    bool isManualMode = digitalRead(MODE_SWITCH) == LOW;
    bool isStopMode = digitalRead(STOP_BUTTON) == LOW;
    if (isStopMode)
    {
        controlMode = MODE_STOP;
        sendCmd("mode 2");
    }
    else if (isManualMode)
    {
        controlMode = MODE_MANUAL;
        sendCmd("mode 1");
    }
    else
    {
        controlMode = MODE_AUTO;
        sendCmd("mode 0");
    }
}

void maybeSendPulse()
{
    if (millis() - lastPulseTime > PULSE_DELAY)
    {
        sendCmd("pulse");
        lastPulseTime = millis();
    }
}

void sendCmd(String cmd)
{
    Serial.println(cmd);
}

void initializeJoysticks()
{
    pinMode(HOE_JOYSTICK_Y, INPUT);
    pinMode(HOE_JOYSTICK_X, INPUT);
    pinMode(DRIVE_JOYSTICK_Y, INPUT);
    pinMode(DRIVE_JOYSTICK_X, INPUT);
}

void initializeButtons()
{
    pinMode(MODE_SWITCH, INPUT_PULLUP);
    pinMode(STOP_BUTTON, INPUT_PULLUP);
}

void setup()
{
    Serial.begin(9600);
    delay(500);
    initializeJoysticks();
    initializeButtons();
}

void loop()
{
    maybeChangeMode();
    maybeSendPulse();
    if (controlMode == MODE_MANUAL)
    {
        sendDriveCmd();
        sendHoeCmd();
        delay(100);
    }
    else if (controlMode == MODE_STOP)
    {
        sendCmd("mode 2");
        delay(1000);
    }
}