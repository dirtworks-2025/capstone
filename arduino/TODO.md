### TODO:

- code RC control (conditional on toggle switch, default automated)
    - electrical stop from RC controller (toggles EN pins on motor controllers)
        - this would be best implemented as a "heartbeat" from the RC controller - sends affirmative signal every 1 second, rover stops driving if it doesn't hear this signal for ~3 seconds
    - two joysticks for RC control of (1) tank drive and (2) gantry and stirrup hoe (only listen to one of these #2 signals at a time - whichever is highest abs)
- raspberry pi serial comms (use CV to drive the rover)
    - not a priority, since our goal for next week is full RC control, but not operational CV
    - probably best to add this to the Arduino logic, even if the raspberry pi side is not implemented
- stirrup hoe control (including encoders)