## Synopsis

This is our code for the p1 project. In this file there will be a short introduction for running the code we have made so far.

Use the **WIKI section** for guides and commands.

### go_to_point:

This node subscribes to the `joystick_control` and reacts to the diffrent user-inputs and fires the nessecary functions.

### joystick_control:

This node will listen for user-input and determine if it is manual steering, go-to command, save location or emergency stop and publishes them to `go_to_point`.

## Installing the code
This guide assumes you are using Ubuntu and have ros-kinetic and catkin packages installed.

Go to your ws and clone the repository
```
git clone git@github.com:SorenMV/rob1b217.git
```
Build the package
```
catkin build
```

## Running the code on the turtlebot

Step 1. Run turtlebot bringup by typing 
```
roslaunch turtlebot_bringup minimal.launch
```
in the terminal (Remember to turn on the kobuki base!!). `roscore` is included in this.


Step 2. Place the turtlebot in coordinate 0.0.0 which is in front of our door in the middle of the yellow line. (else use the pose estimate in RVIZ)


Step 3. Load the premade map by typing 
```
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/rob1b217/ws/maps/hallway2.yaml
```
when it says "odom received" the map is loaded and you're good to go.


(Step 4.) This step is only if you want to see the map on the screen. Open rviz by typing 
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Step 5. Source the setup file
```
source devel/setup.bash
```

Step 6. Run the program

```
rosrun go_to_point go_to_point_node
```
```
rosrun joystick_constrol joystick_constrol_node
```
---
or use the roslaunch command to skip all steps:
```
source devel/setup.bash
roslaunch rosrun go_to_point base.launch
```
## User guide
The robot has 4 predefined locations: __A__, __B__, __X__, __Y__ on the X-Box controller. 

To save a new location press: __LB__+`<button>` where `<button>` is any of the aforementioned buttons on the X-Box controller.
