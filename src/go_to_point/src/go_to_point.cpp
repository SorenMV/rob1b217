#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <sstream>


class GoToPoint
{
private:
	// Initialize variables
	move_base_msgs::MoveBaseGoal goal;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;


	// This function gets called when a coordinate is set and wants to move
	void _go_to_point(const move_base_msgs::MoveBaseGoal& goal)
	{
		//wait for the action server to come up
		while( ! client.waitForServer(ros::Duration(5.0)) )
		{
		    ROS_INFO("Waiting for the move_base action server to come up.");
		}

		// Send the goal to actionlib and return to the "callback" function
		client.sendGoal(goal, boost::bind(&GoToPoint::_target_reached, this, _1));
		ROS_INFO("Navigating ...");
	}

	// This function gets called when actionlib is done navigating to the goal
	void _target_reached(const actionlib::SimpleClientGoalState& state)
	{
		// Check if actionlib successfully navigated to goal
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Target reached.");
		}
		else
		{
			// This can be because of obsticles making it impossible to get to goal.
			ROS_INFO("Failed to reach target.");
		}
	}

	// This function gets called when a topic we subscribe to is recieved.
	// A command with type "String" gets send and it will look-up to see
	// if that string is saved. If it is, set the coordinates and pass them to next function.
	void _command_send(const std_msgs::String& command)
	{
		ROS_INFO("Recived command: %s", command.data.c_str());
		
		// 1. Lookup db for command string and get coordinates
		
		// coordinate frame ("map", "base_link")
		goal.target_pose.header.frame_id = "map";


		// Hard coded coordinates<double>:
		goal.target_pose.pose.position.x = 2.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.z = 0.5;
		goal.target_pose.pose.orientation.w = 0.5;

		// home = [0.0, 0.0]
		// hallway1 = [4.0, 1.0] ?
		// hallway2 = [-3.0, 1.0] ?

		// Send coordinates to next function
		_go_to_point(goal);
	}

public:
	// Constructor
	GoToPoint():
		client("move_base", true)
	{
		// Standard ros NodeHandle
		// ros::NodeHandle n;

		// Make a publish element with type <std_msgs::String>
		// marker_pub = n.advertise<std_msgs::String>("command_send", 1);

		// This is how to subscribe("topic", queueSize, callbackFunction, hint)
		// click_sub = n.subscribe("command_send", 10, &Route::_command_send, this);



		// TODO: Make a fuction that listens for a joystick input

		// This is how to make a string message
		std_msgs::String msg;

     	std::stringstream ss;
     	ss << "Kitchen";
     	msg.data = ss.str();

     	// This sends a command string to next function
		_command_send(msg);
	};
};

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point");

	// Conctruct the class "GoToPoint"
	GoToPoint g;

	return 0;
}