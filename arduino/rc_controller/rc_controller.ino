#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Screen Pins
#define DISPLAY_SDA A4
#define DISPLAY_SCL A5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Joystick pins
#define HOE_JOYSTICK_Y A0
#define HOE_JOYSTICK_X A1
#define DRIVE_JOYSTICK_Y A2
#define DRIVE_JOYSTICK_X A3

// Button pins
#define STOP_MODE_BUTTON 2
#define AUTO_MODE_BUTTON 3
#define REVERSE_AUTO_MODE_BUTTON 4 // might change this button to binary raise/lower stirrup hoe
#define FAST_MODE_BUTTON 5

// Mode state
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_STOP 2
byte controlMode = MODE_MANUAL;

// Delays
#define MAX_DELAY_BETWEEN_CMDS_MS 250 // Maximum delay between two consecutive commands

// Speed limits
#define TANK_DRIVE_SLOW_SPEED_LIMIT 0.5 // Speed limit for the drive joystick (0.0 to 1.0)
#define TANK_DRIVE_FAST_SPEED_LIMIT 1.0 // Speed limit for the drive joystick (0.0 to 1.0)
#define SLOW_STEP_DELAY_uS 10000
#define FAST_STEP_DELAY_uS 3000

// Deadzone
#define DEADZONE_THRESHOLD 0.05 // Deadzone threshold for the joysticks (0.0 to 1.0)

// Radio
#define CE_PIN 7
#define CSN_PIN 8
const byte address[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

// Command types:
// drive <leftSpeed> <rightSpeed>
// gantry <gantryStepDelay_uS> 
// hoe <targetHoePosition>
// mode <number> (0 = auto, 1 = manual, 2 = stop) <FORWARD / BACKWARD> (auto mode only)

void sendDriveCmd()
{
    int x = analogRead(DRIVE_JOYSTICK_X);
    int y = analogRead(DRIVE_JOYSTICK_Y);
    float xNormalized = map(x, 0, 1023, -100, 100) / 100.0;
    float yNormalized = -map(y, 0, 1023, -100, 100) / 100.0;

    // Set deadzone per axis
    if (abs(xNormalized) < DEADZONE_THRESHOLD)
    {
        xNormalized = 0.0;
    }
    if (abs(yNormalized) < DEADZONE_THRESHOLD)
    {
        yNormalized = 0.0;
    }

    // Escape if both axes are in the deadzone
    if (xNormalized == 0.0 && yNormalized == 0.0)
    {
        sendCmd("drive 0 0");
        return;
    }

    // Handle fast mode vs slow mode
    float speedLimit = TANK_DRIVE_SLOW_SPEED_LIMIT;
    if (digitalRead(FAST_MODE_BUTTON) == HIGH)
    {
        speedLimit = TANK_DRIVE_FAST_SPEED_LIMIT;
    }

    // Set tank drive speeds
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
    if (abs(xNormalized) < 100 * DEADZONE_THRESHOLD)
    {
        xNormalized = 0;
    }
    if (abs(yNormalized) < 100 * DEADZONE_THRESHOLD)
    {
        yNormalized = 0;
    }

    // Escape if both axes are in the deadzone
    if (xNormalized == 0 && yNormalized == 0)
    {
        sendCmd("gantry 0");
        return;
    }

    // Only listen to the signal with the greatest magnitude
    if (abs(xNormalized) > abs(yNormalized))
    {
        int stepDelay_uS = map(abs(xNormalized), 0, 100, SLOW_STEP_DELAY_uS, FAST_STEP_DELAY_uS);
        int gantryStepDelay_uS = xNormalized > 0 ? -stepDelay_uS : stepDelay_uS;
        sendCmd("gantry " + String(gantryStepDelay_uS));
    }
    else
    {
        if (yNormalized > 80)
        {
            sendCmd("hoe 0"); // Lower hoe
        }
        else if (yNormalized < -80)
        {
            sendCmd("hoe -70"); // Raise hoe
        }
    }
}

void maybeUpdateControlMode() 
{
    bool isAutoMode = digitalRead(AUTO_MODE_BUTTON) == HIGH;
    bool isStopMode = digitalRead(STOP_MODE_BUTTON) == HIGH;

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
    if (cmd.length() == 0)
    {
        Serial.println("Empty command, not sending.");
        return;
    }
    // Serial.println("Sending command: " + cmd);
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
    pinMode(AUTO_MODE_BUTTON, INPUT);
    pinMode(STOP_MODE_BUTTON, INPUT);
    pinMode(REVERSE_AUTO_MODE_BUTTON, INPUT);
    pinMode(FAST_MODE_BUTTON, INPUT);
}

void initializeRadio()
{
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.display();
  delay(200);
}

void initializeDisplayLayout() {
  display.clearDisplay();

  // Top title
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("ROW-RIDER");

  // Status message box (top right)
  display.drawRect(110, 0, 15, 16, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(93, 4);
  display.print("S");

  // Dashed line separating header
  for (int i = 0; i < SCREEN_WIDTH; i += 4)
    display.drawPixel(i, 17, SSD1306_WHITE);

  // Arrow label
  display.setCursor(0, 20);
  display.print("Drive");

  // Variable text box
  display.drawRect(40, 22, 84, 40, SSD1306_WHITE); // large box
  display.setCursor(45, 30);
  display.setTextSize(1);
  display.print("");

  display.display();
}

void updateStatus(uint8_t mode, String txt, int angle) {
  String statusText;
  switch (mode) {
    case 0: statusText = "AUTO"; break;
    case 1:  statusText = "MAN";  break;
    case 2: statusText = "STOP"; break;
    default:        statusText = "???";  break;
  }

  display.fillRect(127, 1, 40, 14, SSD1306_BLACK); // clear box
  display.setCursor(93, 4);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(statusText);
  updateVariableText(txt);
  updateArrow(angle);
  display.display();
}

void updateVariableText(String txt) {
  display.fillRect(41, 23, 82, 38, SSD1306_BLACK); // Clear box area

  int16_t x1, y1;
  uint16_t w, h;

  // Try text size 2
  display.setTextSize(2);
  display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);

  if (w > 80 || h > 38) {
    // If too big, fallback to size 1
    display.setTextSize(1);
    display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  }

  // Still too wide? Crop the string (optional)
  while (w > 80 && txt.length() > 0) {
    txt.remove(txt.length() - 1);
    display.getTextBounds(txt + "...", 0, 0, &x1, &y1, &w, &h);
  }

  // Center it
  int x = 40 + (84 - w) / 2;
  int y = 22 + (40 - h) / 2;

  display.setCursor(x, y);
  display.setTextColor(SSD1306_WHITE);
  display.print(txt);
  display.display();
}


void updateArrow(int angle) {
  // Clear old arrow
  display.fillRect(0, 30, 30, 34, SSD1306_BLACK);

  float rad = angle * PI / 180.0;
  int x0 = 15; // origin
  int y0 = 60;
  int len = 20;
  int x1 = x0 + len * sin(rad);
  int y1 = y0 - len * cos(rad);

  display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  display.display();
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    initializeRadio();
    initializeJoysticks();
    initializeButtons();
    // initializeDisplay();
    // initializeDisplayLayout();
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
        if (digitalRead(REVERSE_AUTO_MODE_BUTTON) == HIGH)
        {
            sendCmd("mode 0 BACKWARD");
        }
        else
        {
            sendCmd("mode 0 FORWARD");
        }
        delay(MAX_DELAY_BETWEEN_CMDS_MS);
        // updateStatus(controlMode, "Autonomous Trackging Started", 0);
    }
    else if (controlMode == MODE_MANUAL)
    {
        sendCmd("mode 1");
        delay(10);
        sendDriveCmd();
        delay(10);
        sendHoeCmd();
        delay(10);
        // updateStatus(controlMode, "Put Stuff Here", 0);
    }
    else if (controlMode == MODE_STOP)
    {
        sendCmd("mode 2");
        delay(MAX_DELAY_BETWEEN_CMDS_MS);
        // updateStatus(controlMode, "STOPPED", 0);
    } else {
        Serial.println("Error: Unrecognized control mode.");
        // updateStatus(controlMode, "BROKEN", 0);
    }

}
