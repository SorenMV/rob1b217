#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <sstream>


class Route
{
private:
	move_base_msgs::MoveBaseGoal goal;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	ros::Publisher marker_pub;
	ros::Subscriber click_sub;

	void _go_to_point(const move_base_msgs::MoveBaseGoal& goal_point)
	{
		//wait for the action server to come up
		while(!client.waitForServer(ros::Duration(5.0))){
		    ROS_INFO("Waiting for the move_base action server to come up");
		}


		client.sendGoal(goal_point, boost::bind(&Route::_target_reached_cb, this, _1));
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
		
		// 1. Lookup db for command string and get coordinates
		
		// coordinate frame ("map", "base_link")
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		// Hard coded coordinates<double>:
		goal.target_pose.pose.position.x = 2.0;
		goal.target_pose.pose.position.y = 1.0; 
		goal.target_pose.pose.orientation.z = 0.5; 
		goal.target_pose.pose.orientation.w = 0.5;

		// home = [0.654, -1.07]
		// hallway = [4.0, -3.0]

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
	ros::init(argc, argv, "go_to_point_smv");

	// Conctruct the class "Route"
	Route r;

	ros::spin();
	return 0;
}
