#include <ros/ros.h>
//moving the turtlebot
#include <geometry_msgs/Twist.h>
//getting joy input
#include <sensor_msgs/Joy.h>


class joystick_class
{
    //public because joystick_class() constructor is called outside
public:
  joystick_class();

private:
  void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);
 
  ros::NodeHandle joy_nodehandle;
  ros::Subscriber joy_subscriber;
  ros::Publisher joy_publisher;
//declaring variable "joy_cmd_vel"
  geometry_msgs::Twist joy_cmd_vel;

//is 'button' pressed? variables
  bool A=false, B, X, Y, LB, RB, back, start, power, LJ, RJ;

//speed control  
  float linear_velocity, angular_velocity;
};









//contructor
joystick_class::joystick_class()
{
    joy_subscriber = joy_nodehandle.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_class::joystick_callback, this);
//command_velocity
    joy_publisher = joy_nodehandle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
};











void joystick_class::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{

//teleop
////TO DO: subscribe to sensor and stop if needed
linear_velocity = 0.5;
angular_velocity = 0.5;

joy_cmd_vel.angular.z = angular_velocity * joy->axes[0];
joy_cmd_vel.linear.x = linear_velocity * joy->axes[1];
joy_publisher.publish(joy_cmd_vel);


//buttons

if(A == false && joy->buttons[0] == 1)
{
    ROS_INFO("YEEE!");




    A = true;
}

if(A == true && joy->buttons[0] == 0 )
{
    A = false;
}











}










int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");

// Conctruct the class "joystick_class"
  joystick_class Candy;

// Repeat receiving subscribtion, thus executing callback function
  ros::spin();
}