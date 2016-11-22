#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;

ros::NodeHandle nh;
ros::Publisher stop_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);




int main(int argc, char** argv){
  ros::init(argc, argv, "emgstop");
  





  ROS_INFO("HELLO WORD 1 !");
  geometry_msgs::Twist stop;

  int stop_activated = 0;
  cin >> stop_activated;
  
  if (stop_activated == 1)
  {

  	stop.linear.x = 0;
  	stop.linear.y = 0;
  	stop.angular.z = 0;
  	
  	
  	stop_pub.publish(stop);

  }

  
  
  return 0;
}