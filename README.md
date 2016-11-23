## Synopsis

This is our code for the p1 project. In this file there will be a short introduction for running the code we have made so far.

**Use the WIKI section for guides and commands.**

## go_to_point:

This is the code we will use.

## Running the code on the turtlebot (Remember to source the workspace!)

Step 1. Run a roscore by typing `roscore` in the terminal.


Step 2. Run turtlebot bringup by typing 
```
roslaunch turtlebot_bringup minimal.launch
```
in the terminal (Remember to turn on the kobuki base!!)


Step 3. Place the turtlebot in coordinate 0.0.0 which is in front of our door in the middle of the yellow line. (else use the pose estimate in RVIZ)


Step 4. Load the premade map by typing 
```
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/rob1b217/ws/maps/hallway2.yaml
```
when it says "odom received" the map is loaded and you're good to go.


(Step 5.) This step is only if you want to see the map on the screen. Open rviz by typing 
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```


Step 6. Run the program by typing:
```

rosrun go_to_point go_to_point_node
```

Step 7. The robot has 4 predefined locations: A, B, X, Y. 


Step 8. When it says "Press a key" you can press either A, B, X, Y and the robot will go to the defined location.
