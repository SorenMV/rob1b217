#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/UInt16.h"
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


	//Struct for individual locations are created.
	struct DBstruct
	{
		string name; 
		uint16_t key;
		double x, y, z, w;
	};

	// Array with type DBstruct created
	struct DBstruct db[db_size];

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
		// Check if actionlib "succeeded" navigating to the goal
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

	int _init_db()
	{
		//Calling the database and naming it inputFile.
		ifstream inputFile("database/location_database.txt");
		string line;

		//Making sure it has succesfully been opened.
		if(inputFile.is_open())
		{	
			int i = 0;

			//Inputting data from the database into the array. eof=end of file. getline means that it reads the entire line.
			while(!inputFile.eof())
			{
				getline(inputFile, line);
				stringstream ss (line);
				ss >> db[i].name;
				ss >> db[i].key;
				ss >> db[i].x;
				ss >> db[i].y;
				ss >> db[i].z;
				ss >> db[i].w;
				
				i++;
			}
			inputFile.close();
			return 1;
		}
		return 0;
	}

	// This function gets called when a key is pressed
	// A command with type "string" gets send and it will look-up to see
	// if that string is saved.

	void _callback_from_joy(const std_msgs::UInt16 subscribed_key)
	{
		//Lookup db for subscribed_key and get coordinates. Line by line from the top. 
		for (int i = 0; i < db_size; ++i)
		{
			if(db[i].key == subscribed_key.data)
			{	
				//Feeding coordinations to goal from the specific array in the db array:
				goal.target_pose.pose.position.x = db[i].x;
				goal.target_pose.pose.position.y = db[i].y;
				goal.target_pose.pose.orientation.z = db[i].z;
				goal.target_pose.pose.orientation.w = db[i].w;

				//"cout" the coordinates and the name of the location
				ROS_INFO("Going to: %s (%f, %f, %f, %f)", 
					db[i].name.c_str(), db[i].x, db[i].y, db[i].z, db[i].w);

				// Send coordinates to next function
				_go_to_point(goal);
			}
		}
	}

public:
	// Constructor
	GoToPoint():
		client("move_base")
	{
		_init_db();

		// coordinate frame. "map" or "base_link"
		goal.target_pose.header.frame_id = "map";

		// Subscribing to our joystick topic
		subscribtion_from_joy = go_to_point_nodehandle.subscribe<std_msgs::UInt16>("go_to_point_trigger", 10, &GoToPoint::_callback_from_joy, this); 
		
		ROS_INFO("Started. Listening for commands ...");
	}
};

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point");

	// Conctruct the class "GoToPoint"
	GoToPoint goTo;

	ros::spin();
	return 0;
}