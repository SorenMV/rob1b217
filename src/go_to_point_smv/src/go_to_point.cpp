#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
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

	// This function gets called when a coordinate is set and wants to move.
	void _go_to_point(const move_base_msgs::MoveBaseGoal& goal_point)
	{
		//wait for the action server to come up
		while(!client.waitForServer(ros::Duration(5.0)))
		{
		    ROS_INFO("Waiting for the move_base action server to come up.");
		}

		// Send the goal to actionlib
		client.sendGoal(goal_point, boost::bind(&Route::_target_reached_cb, this, _1));
		ROS_INFO("Navigating ...");
	}

	// This function gets called when actionlib is done navigating to the goal.
	void _target_reached_cb(const actionlib::SimpleClientGoalState& state)
	{
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Target reached.");
		}
		else
		{
			ROS_INFO("Failed to reach target.");
			// This can be because of obsticles making it impossible to get to goal.
		}
	}

	// This function gets called when a topic we subscribe to is recieved.
	// A command with type "String" gets send and it will look-up to see
	// if that string is saved. If it is, set the coordinates and pass them to next function.
	void _command_send_cb(const std_msgs::String& command)
	{
		ROS_INFO("Recived command: %s", command.data.c_str());
		
		// 1. Lookup db for command string and get coordinates
		
		// coordinate frame ("map", "base_link")
		goal.target_pose.header.frame_id = "map";

		// Hard coded coordinates<double>:
		goal.target_pose.pose.position.x = 4.0;
		goal.target_pose.pose.position.y = -3.0;
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
		// marker_pub = n.advertise<std_msgs::String>("command_send", 1);

		// subscribe("topic", queueSize, callbackFunction, hint)
		// click_sub = n.subscribe("command_send", 10, &Route::_command_send_cb, this);



		// TODO: Make a fuction that listens for a joystick input

		// This is how you make a string message
		std_msgs::String msg;

     	std::stringstream ss;
     	ss << "Kitchen";
     	msg.data = ss.str();

     	// This send a command string to next function.
		_command_send_cb(msg);
	};
};

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point_smv");

	// Conctruct the class "Route"
	Route r;

	return 0;
}