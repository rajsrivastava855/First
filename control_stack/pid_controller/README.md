# PID CONTROL
This task is for Ateet.

## DEADLINES
First review: 29th December

Final deadline: 31st December


### TASK 1
Make a UI with 6 tabs
- Heave
- Sway
- Surge
- Yaw
- Pitch
- Roll

### TASK 2
Each tab contains 3 or 4 sliders
- Kp
- Ki
- Kd
- Setpoint (Not for pitch and roll) (heave and yaw setpoint - integer, rest float)

Sliders should operate exactly like they do in the package thruster_controller.
Think how float values can be included in sliders.

### TASK 3
The Kp, Ki, Kd values for each parameter will be published on topics
- `/sway_params`
- `/heave_params`
- and so on

The setpoints will be published together in the topic `/setpoints`

For the next two tasks, you may refer to `thruster_controller` package:

### TASK 4
On startup, the node will read configuration from files in `config` folder.
The file has order kp, ki, kd, setpoint for each parameter.

### TASK 5
There will be a save button which will save pid values and setpoints to the
`config` folder again.

### TASK 6
Understand what the `hammerhead` package does. Hint: it has been used in `thruster_controller`

`#include <hammerhead/hammerhead.h>`

For all tasks, ensure same file and variable naming convention that has been used
in `thruster_controller` package. Also ensure same folder organization for include
folder.
