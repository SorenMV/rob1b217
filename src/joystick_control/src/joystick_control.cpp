#include <ros/ros.h>
//getting joy input
#include <sensor_msgs/Joy.h>
//moving the turtlebot
#include <geometry_msgs/Twist.h>
//publishing to go_to_point node
#include "std_msgs/UInt16.h"


class joystick_class
{
//public because joystick_class() constructor is called in main function (everything can be public)
public:
  joystick_class();

private:
//callback function executed each time "sensor_msgs/Joy" node publishes something we are subscribed to
  void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);
 
  ros::NodeHandle       joy_nodehandle      ;
  ros::Subscriber       joy_subscriber      ;
  ros::Publisher        joy_move_base_pub   ;
  ros::Publisher        joy_go_to_point_pub ;

//declaring publishing variables
  geometry_msgs::Twist  joy_cmd_vel         ;
  std_msgs::UInt16      goal_target         ;

//is button pressed? variables
  bool Apressed, Bpressed, Xpressed, Ypressed, LBpressed, RBpressed, backpressed, startpressed, powerpressed, LJpressed, RJpressed;

//checks whether the kobuki base is turning
  bool is_turning;

//speed control variables 
  float smoother, temp_axes0;

//play with values if you dare!
  const float deadman_radius        = 0.5   ;
  const float linear_velocity       = 0.05  ;
  const float angular_velocity      = 0.25  ;
  const float smoother_value        = 1.015 ;
  const float smoother_peak         = 6     ;
  const float is_turning_value      = 1.2   ;
  const float additional_slow_rate  = 1.001 ;
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
  joy_go_to_point_pub = joy_nodehandle.advertise<std_msgs::UInt16>("go_to_point_trigger", 1);


  smoother = smoother_value;
};


void joystick_class::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
//MANUAL STEERING:
  ROS_INFO("Linear:   %f",  joy->axes[1]);
  ROS_INFO("Angular:  %f",  joy->axes[0]);
  ROS_INFO("smoother: %f",  smoother    );

//acceleration
if (smoother < smoother_peak && (joy->axes[0] > deadman_radius || joy->axes[0] < -deadman_radius || joy->axes[1] > deadman_radius || joy->axes[1] < -deadman_radius))
{
  smoother = smoother * smoother_value;
}

//decelerattion
if(smoother > smoother_value && joy->axes[0] <= deadman_radius && joy->axes[0] >= -deadman_radius && joy->axes[1] <= deadman_radius && joy->axes[1] >= -deadman_radius)
{
  smoother = smoother / smoother_value;
}


if (joy->axes[0] > deadman_radius || joy->axes[0] < -deadman_radius || joy->axes[1] > deadman_radius || joy->axes[1] < -deadman_radius)
{
  //publishing velocity
  joy_cmd_vel.angular.z = smoother * angular_velocity * joy->axes[0];
  joy_cmd_vel.linear.x = smoother * linear_velocity * joy->axes[1];
  joy_move_base_pub.publish(joy_cmd_vel);

  //sending cancelGoal request
  goal_target.data = 8;  
  joy_go_to_point_pub.publish(goal_target);
  
  temp_axes0 = joy->axes[1];

  //is kobuki turning?
  if (joy->axes[0] >= (is_turning_value * deadman_radius) || joy->axes[0] <= -(is_turning_value * deadman_radius))
  {
   is_turning=true;
  }
  if(joy->axes[0] < (is_turning_value * deadman_radius) && joy->axes[0] > -(is_turning_value * deadman_radius)) 
  {
    is_turning = false;
  } 
}


//smooth deceleration
if(smoother > smoother_value && joy->axes[0] <= deadman_radius && joy->axes[0] >= -deadman_radius && joy->axes[1] <= deadman_radius && joy->axes[1] >= -deadman_radius)
{
  temp_axes0 = temp_axes0 / additional_slow_rate;
  joy_cmd_vel.linear.x = smoother * linear_velocity * temp_axes0;
  joy_move_base_pub.publish(joy_cmd_vel);
 

//turning stop
  if(is_turning == true)  
  {
    joy_cmd_vel.angular.z = 0;
    joy_move_base_pub.publish(joy_cmd_vel);
  }
}





//BUTTONS:

//Location buttons
//RB
if(joy->buttons[5] == 0)
{
  //A
  if(Apressed == false && joy->buttons[0] == 1)
  {
  goal_target.data = 0;
  joy_go_to_point_pub.publish(goal_target);
  Apressed = true;
  }
  if(Apressed == true && joy->buttons[0] == 0){Apressed = false;}


  //B
  if(Bpressed == false && joy->buttons[1] == 1)
  {
    goal_target.data = 1;
    joy_go_to_point_pub.publish(goal_target);
    Bpressed = true;
  }
  if(Bpressed == true && joy->buttons[1] == 0){Bpressed = false;}


  //X
  if(Xpressed == false && joy->buttons[2] == 1)
  {
    goal_target.data = 2;
    joy_go_to_point_pub.publish(goal_target);
    Xpressed = true;
  }
  if(Xpressed == true && joy->buttons[2] == 0){Xpressed = false;}


  //Y
  if(Ypressed == false && joy->buttons[3] == 1)
  {
    goal_target.data = 3;
    joy_go_to_point_pub.publish(goal_target);
    Ypressed = true;
  }
  if(Ypressed == true && joy->buttons[3] == 0){Ypressed = false;}
}


//Combination of 2 buttons to save location
//RB
else if(joy->buttons[5] == 1)
  {
  //A+RB
  if(Apressed == false && joy->buttons[0] == 1)
  {
    goal_target.data = 4;
    joy_go_to_point_pub.publish(goal_target);
    Apressed = true;
  }
  if(Apressed == true && joy->buttons[0] == 0){Apressed = false;}


  //B+RB
  if(Bpressed == false && joy->buttons[1] == 1)
  {
    goal_target.data = 5;
    joy_go_to_point_pub.publish(goal_target);
    Bpressed = true;
  }
  if(Bpressed == true && joy->buttons[1] == 0){Bpressed = false;}


  //X+RB
  if(Xpressed == false && joy->buttons[2] == 1)
  {
    goal_target.data = 6;
    joy_go_to_point_pub.publish(goal_target);
    Xpressed = true;
  }
  if(Xpressed == true && joy->buttons[2] == 0){Xpressed = false;}


  //Y+RB
  if(Ypressed == false && joy->buttons[3] == 1)
  {
    goal_target.data = 7;
    joy_go_to_point_pub.publish(goal_target);
    Ypressed = true;
  }
  if(Ypressed == true && joy->buttons[3] == 0){Ypressed = false;}
}



//emergency stop + cancelGoal 
//BACK button
if(backpressed == false && joy->buttons[6] == 1)
{
  joy_cmd_vel.angular.z = 0;
  joy_cmd_vel.linear.x = 0;
  joy_move_base_pub.publish(joy_cmd_vel);
  //sending cancelGoal request
  goal_target.data = 8;  
  joy_go_to_point_pub.publish(goal_target);
  backpressed = true;
}
if(backpressed == true && joy->buttons[6] == 0){backpressed = false;}


/*
//LB
if(LBpressed == false && joy->buttons[4] == 1)
{
  LBpressed = true;
}
if(LBpressed = true && joy->buttons[4] == 0){LBpressed = false;}


//START button
if(startpressed == false && joy->buttons[7] == 1)
{ 
  startpressed = true;
}
if(startpressed == true && joy->buttons[7] == 0){startpressed = false;}


//POWER button
if(powerpressed == false && joy->buttons[8] == 1)
{   
  powerpressed = true;
}
if(powerpressed == true && joy->buttons[8] == 0){powerpressed = false;}


//LJ
if(LJpressed == false && joy->buttons[9] == 1)
{   
  LJpressed = true;
}
if(LJpressed == true && joy->buttons[9] == 0){LJpressed = false;}


//RJ
if(RJpressed == false && joy->buttons[10] == 1)
{
  RJpressed = true;
}
if(RJpressed == true && joy->buttons[10] == 0){RJpressed = false;}
*/
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");
  
// Conctruct the class "joystick_class"
  joystick_class Candy;

// Repeat receiving subscribtion, thus executing callback function
  ros::spin();
}










// rosparam set joy_node/autorepeat_rate "200"