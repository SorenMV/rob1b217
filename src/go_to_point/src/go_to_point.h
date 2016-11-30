#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/UInt16.h"
#include "tf/transform_listener.h"

#ifndef GO_TO_POINT_INCLUDE
#define GO_TO_POINT_INCLUDE

/* Your function statement here */

#define db_size 10

using namespace std;
class GoToPoint
{
private:
	// Initialize variables
	ros::NodeHandle go_to_point_nodehandle;
	move_base_msgs::MoveBaseGoal goal;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	ros::Subscriber subscribtion_from_joy;
	tf::TransformListener tf_listener;
	tf::StampedTransform tf_stamped;

	//Struct for individual locations are created.
	struct DBstruct
	{
		string name; 
		uint16_t key;
		double x, y, z, w;
	};

	// Array with type DBstruct created
	struct DBstruct db[db_size];
	
	int _go_to_point(const move_base_msgs::MoveBaseGoal& goal);

	void _target_reached(const actionlib::SimpleClientGoalState& state);

	void _init_db();

	void _callback_from_joy_to_send_goal(const std_msgs::UInt16 );
	
public:
	// Constructor
		// Constructor
	GoToPoint():
		client("move_base")
	{
		_init_db();

		// coordinate frame. "map" or "base_link"
		goal.target_pose.header.frame_id = "map";

		// Subscribing to our joystick topic
		subscribtion_from_joy = go_to_point_nodehandle.subscribe<std_msgs::UInt16>("go_to_point_trigger", 10, &GoToPoint::_callback_from_joy_to_send_goal, this); 
		
		ROS_INFO("Started. Listening for commands ...");


	}

};


#endif
