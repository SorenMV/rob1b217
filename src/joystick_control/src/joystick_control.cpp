#include <ros/ros.h>
//getting joy input
#include <sensor_msgs/Joy.h>
//moving the turtlebot
#include <geometry_msgs/Twist.h>
//publishing to go_to_point node
#include "std_msgs/UInt8.h"


class joystick_class
{
//public because joystick_class() constructor is called in main function (everything can be public)
public:
  joystick_class();

private:
//callback function executed each time "sensor_msgs/Joy" node publishes something we are subscribed to
  void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);
 
  ros::NodeHandle joy_nodehandle;
  ros::Subscriber joy_subscriber;
  ros::Publisher joy_move_base_pub;
  ros::Publisher joy_go_to_point_pub;

//declaring publishing variables
  geometry_msgs::Twist joy_cmd_vel;
  std_msgs::UInt8 goal_target;

//is button pressed? variables
  bool Apressed, Bpressed, Xpressed, Ypressed, LBpressed, RBpressed, backpressed, startpressed, powerpressed, LJpressed, RJpressed;

//speed control variables 
  float linear_velocity, angular_velocity;

};

//contructor
joystick_class::joystick_class()
{
//"joy" is the topic we are subscribed to
//"mobile_base/commands/velocity" is a topic controlling the movement if the mobile base
//"this" is used for passing a reference to the instance of the object (in this case it is the class itself)
    joy_subscriber = joy_nodehandle.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_class::joystick_callback, this); 
    joy_move_base_pub = joy_nodehandle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

//publising our own topic "go_to_point_trigger" to trigger the "go_to_point" node
    joy_go_to_point_pub = joy_nodehandle.advertise<std_msgs::UInt8>("go_to_point_trigger", 1);
};


void joystick_class::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy) //note that joy is pointer
{
//teleop
linear_velocity = 0.5;
angular_velocity = 1.5;
////TO DO: test these values

joy_cmd_vel.angular.z = angular_velocity * joy->axes[0];
joy_cmd_vel.linear.x = linear_velocity * joy->axes[1];
joy_move_base_pub.publish(joy_cmd_vel);
////TO DO: subscribe to sensor and stop if needed
////TO DO: smooth the movement?


//buttons
//A
if(Apressed == false && joy->buttons[0] == 1)
{
    goal_target.data=0;
    joy_go_to_point_pub.publish(goal_target);
    Apressed = true;
}
if(Apressed == true && joy->buttons[0] == 0){Apressed = false;}

//B
if(Bpressed == false && joy->buttons[1] == 1)
{
    goal_target.data=1;
    joy_go_to_point_pub.publish(goal_target);
    Bpressed = true;
}
if(Bpressed == true && joy->buttons[1] == 0){Bpressed = false;}

//X
if(Xpressed == false && joy->buttons[2] == 1)
{
    goal_target.data=2;
    joy_go_to_point_pub.publish(goal_target);
    Xpressed = true;
}
if(Xpressed == true && joy->buttons[2] == 0){Xpressed = false;}

//Y
if(Ypressed == false && joy->buttons[3] == 1)
{
    goal_target.data=3;
    joy_go_to_point_pub.publish(goal_target);
    Ypressed = true;
}
if(Ypressed == true && joy->buttons[3] == 0){Ypressed = false;}

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");

// Conctruct the class "joystick_class"
  joystick_class Candy;

// Repeat receiving subscribtion, thus executing callback function
  ros::spin();
}