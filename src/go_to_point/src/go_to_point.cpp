#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;


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




	int _command_send(const std_msgs::String& command)
	{
		ROS_INFO("Recived command: %s", command.data.c_str());
		
		// 1. Lookup db for command string and get coordinates
		
		    int location_number;
		    double x_db, y_db, z_db, w_db;

    		ROS_INFO("Location nr. \n 1: Home \n 2: Washroom \n 3: Living room \n 4: Kitchen \n Please choose location");
    		cin >> location_number;

			    ifstream inputFile("/home/rob1b217/ws/rob1b217_w_db/src/go_to_point/src/location_database.txt");
			    string line;

			    for(int lineno = location_number; getline (inputFile,line) && lineno < 5; lineno--)
			        if (lineno == 1)
			        {

			        istringstream ss(line);
			         
			            ss >> x_db >> y_db >> z_db >> w_db;
			            ROS_INFO("reading (%f, %f, %f, %f)", x_db, y_db, z_db, w_db);
					}


		// coordinate frame ("map", "base_link")
		goal.target_pose.header.frame_id = "map";


		// Hard coded coordinates<double>:
		goal.target_pose.pose.position.x = x_db;
		goal.target_pose.pose.position.y = y_db;
		goal.target_pose.pose.orientation.z = z_db;
		goal.target_pose.pose.orientation.w = w_db;

		// home = [0.0, 0.0]
		// hallway1 = [4.0, 1.0] ?
		// hallway2 = [-3.0, 1.0] ?


		ROS_INFO("Sending (%f, %f, %f, %f)", x_db, y_db,z_db,w_db);
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

     	stringstream ss;
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