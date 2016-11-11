#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <sstream>


class Route
{
private:
	geometry_msgs::PointStamped goal;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	ros::Publisher marker_pub;
	ros::Subscriber click_sub;

	void _go_to_point(const geometry_msgs::PointStamped& goal_point)
	{
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = goal_point.header.frame_id;
		goal.target_pose.pose.position = goal_point.point;
		client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1));
		ROS_INFO("Navigating ...");
	}

	void _target_reached_cb(const actionlib::SimpleClientGoalState& state)
	{

		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Target reached.");
		}
		else
		{
			ROS_INFO("Failed to reach target.");
		}
	}

	
	void _command_send_cb(const std_msgs::String& msg)
	{
		ROS_INFO("Recived command: %s", msg.data.c_str());
		
		// 1. Lookup db to associate a command string with coordinates
		
		// coordinate frame ("map", "base_link")
		goal.header.frame_id = "map";

		// Hard coded coordinates:
		goal.point.x = 4.0; // double
		goal.point.y = -3.0; // double
		goal.point.z = 1.0; // double

		// home = [0.654, -1.07, 1.0]
		// random point in hallway = [4.0, -3.0, 1.0]

		// Send coordinates to next function
		_go_to_point(goal);
	}

public:
	// Constructor
	Route():
		client("move_base", true)
	{
		// Standard ros NodeHandle
		ros::NodeHandle n;

		// Make a publish element with type <std_msgs::String>
		// marker_pub = n.advertise<std_msgs::String>("command_send_test", 1);

		// subscribe("topic", queueSize, callbackFunction, hint)
		click_sub = n.subscribe("commandtest", 10, &Route::_command_send_cb, this);

		std_msgs::String msg;
 
     	std::stringstream ss;
     	ss << "hello world";
     	msg.data = ss.str();

		_command_send_cb(msg);
	};
};

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point");

	// Conctruct the class "Route"
	Route r;

	ros::spin();
	return 0;
}
