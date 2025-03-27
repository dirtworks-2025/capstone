### TODO:

- raspberry pi serial comms (use CV to drive the rover)
    - not a priority, since our goal for next week is full RC control, but not operational CV
    - probably best to add this to the Arduino logic, even if the raspberry pi side is not implemented
- stirrup hoe control (including encoders)
- persist CV settings in json file on disk & snychronize w/ frontend on html load
- experiment with anisotropic image morphology kernels - maybe build this as a grid into frontend as well, if I can't hardcode something that works for all cases and need it be a variable for field testing
- quantify uncertainty of driptape regression, so that I can use this as a metric for automatic ideal filter settings selection