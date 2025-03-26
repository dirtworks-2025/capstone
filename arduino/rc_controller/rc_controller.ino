#include <Arduino.h>

// Joystick pins
#define GANTRY_JOYSTICK_Y A0
#define GANTRY_JOYSTICK_X A1
#define TANK_JOYSTICK_Y A2
#define TANK_JOYSTICK_X A3

// Command types:
// tank <leftSpeed> <rightSpeed>
// gantry <speed>

void maybeSendTankCmd()
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

void maybeSendGantryCmd() 
{
    int x = analogRead(GANTRY_JOYSTICK_X);
    int xNormalized = map(x, 0, 1023, -100, 100);

    // Set deadzone
    if (abs(xNormalized) < 10)
    {
        gantryStepDelayMs = 0;
        return;
    }

    int stepDelay = map(abs(xNormalized), 0, 100, 5000, 1500);
    gantryStepDelayMs = xNormalized > 0 ? stepDelay : -stepDelay;
}