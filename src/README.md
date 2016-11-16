## Synopsis

This is our code for the p1 project. In this file there will be a short introduction for running the code we have made so far.

## go_to_point_smv:

This is the code we will use.

## go_to_point_csa:

This is a simplified version of the code, may be good for understanding ROS concepts as well as the coordinate frame. (For some reason it can't compile right now, but the code should me fine)

## Running the code on the turtlebot (Remember to source the workspace!)

Step 1. Run a roscore by typing "roscore" in the terminal.
step 2. Run turtlebot bringup by typing "roslaunch turtlebot_bringup minimal.launch" in the terminal (Remember to turn on the kobuki base!!)
step 3. Place the turtlebot in coordinate 0.0.0 which is in front of our door in the middle of the yellow line. (else use the pose estimate in RVIZ)
step 4. Load the premade map by typing "roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/rob1b217/ws/maps/hallway2.yaml" when it says "odom received" the map is loaded and you're good to go.
(step 5.) This step is only if you want to see the map on the screen. Open rviz by typing "roslaunch turtlebot_rviz_launchers view_navigation.launch"
step 6. Run the robot to the hardcoded location by typing "rosun go_to_point_smv go_to_point_smv_node"

## API Reference

Depending on the size of the project, if it is small and simple enough the reference docs can be added to the README. For medium size to larger projects it is important to at least provide a link to where the API reference docs live.

## Tests

Describe and show how to run the tests with code examples.

## Contributors

Let people know how they can dive into the project, include important links to things like issue trackers, irc, twitter accounts if applicable.

## License

A short snippet describing the license (MIT, Apache, etc.)