#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Joystick pins
#define HOE_JOYSTICK_Y A0
#define HOE_JOYSTICK_X A1
#define DRIVE_JOYSTICK_Y A2
#define DRIVE_JOYSTICK_X A3

// Button pins
#define STOP_BUTTON 2
#define MODE_BUTTON 3
#define BUTTON_3 4
#define BUTTON_4 5

// Mode state
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_STOP 2
byte controlMode = MODE_MANUAL;

// Delays
#define MAX_DELAY_BETWEEN_CMDS_MS 1000 // Maximum delay between two consecutive commands

// Speed limits
#define TANK_DRIVE_SPEED_LIMIT 0.6 // Speed limit for the drive joystick (0.0 to 1.0)
#define SLOW_STEP_DELAY_uS 10000
#define FAST_STEP_DELAY_uS 3000

// Radio
#define CE_PIN 7
#define CSN_PIN 8
const byte address[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

// Command types:
// drive <leftSpeed> <rightSpeed>
// hoe <gantryStepDelay_uS> <upDownSpeed>
// mode <number> (0 = auto, 1 = manual, 2 = stop)

void sendDriveCmd()
{
    int x = analogRead(DRIVE_JOYSTICK_X);
    int y = analogRead(DRIVE_JOYSTICK_Y);
    float xNormalized = map(x, 0, 1023, -100, 100) / 100.0;
    float yNormalized = -map(y, 0, 1023, -100, 100) / 100.0;

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
    int rightTankDriveSpeed = (yNormalized + xNormalized) * (255 / 2) * TANK_DRIVE_SPEED_LIMIT;
    int leftTankDriveSpeed = (yNormalized - xNormalized) * (255 / 2) * TANK_DRIVE_SPEED_LIMIT;
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
        int stepDelay_uS = map(abs(xNormalized), 0, 100, SLOW_STEP_DELAY_uS, FAST_STEP_DELAY_uS);
        int gantryStepDelay_uS= xNormalized > 0 ? stepDelay_uS : -stepDelay_uS;
        sendCmd("hoe " + String(gantryStepDelay_uS) + " 0");
    }
    else
    {
        int upDownSpeed = map(abs(yNormalized), 0, 100, 0, 255);
        int upDownDir = yNormalized > 0 ? 1 : -1;
        sendCmd("hoe 0 " + String(upDownSpeed * upDownDir));
    }
}

void maybeUpdateControlMode() 
{
    bool isAutoMode = digitalRead(MODE_BUTTON) == HIGH;
    bool isStopMode = digitalRead(STOP_BUTTON) == HIGH;

    if (isStopMode)
    {
        controlMode = MODE_STOP;
    }
    else if (isAutoMode)
    {
        controlMode = MODE_AUTO;
    }
    else
    {
        controlMode = MODE_MANUAL;
    }
}

void sendCmd(String cmd)
{
    radio.write(cmd.c_str(), cmd.length() + 1);
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
    pinMode(MODE_BUTTON, INPUT);
    pinMode(STOP_BUTTON, INPUT);
    pinMode(BUTTON_3, INPUT);
    pinMode(BUTTON_4, INPUT);
}

void initializeRadio()
{
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    initializeRadio();
    initializeJoysticks();
    initializeButtons();
}

void loop()
{
    maybeUpdateControlMode();
    delayMicroseconds(100);

    // Send at least one command per second as a "pulse"
    // The robot will stop if no commands are received for 3 seconds
    // Basically, the controller "reminds" the robot the mode it's in,
    // if it doesn't have anything else to say

    if (controlMode == MODE_AUTO)
    {
        sendCmd("mode 0");
        delay(1000);
    }
    else if (controlMode == MODE_MANUAL)
    {
        sendCmd("mode 1");
        delay(40);
        sendDriveCmd();
        delay(30);
        sendHoeCmd();
        delay(30);
    }
    else if (controlMode == MODE_STOP)
    {
        sendCmd("mode 2");
        delay(1000);
    } else {
        Serial.println("Error: Unrecognized control mode.");
    }
}