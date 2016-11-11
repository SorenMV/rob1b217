#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"


class Route
{
private:
	unsigned int stops_initialized;
	geometry_msgs::PointStamped goal;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	ros::Publisher marker_pub;
	ros::Subscriber click_sub;

	void _go_to_point(const geometry_msgs::PointStamped& goal_point)
	{
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = goal_point.header.frame_id;
		goal.target_pose.pose.position = goal_point.point;
		client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2));
	}

	void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		// std::swap(from, goal);
		// _send_goal(goal);
	}

	
	void _command_send_cb(const std_msgs::String& msg)
	{
		ROS_INFO("Recived command: %s", msg.data.c_str());
		
		// 1. Lookup db to associate a command string with coordinates
		
		// coordinate frame
		goal.header.frame_id = "map"; 

		// Hard coded coordinates:
		goal.point.x = 4; // double
		goal.point.y = -3; // double
		goal.point.z = 1; // double

		// home = [0.654, -1.07, 1]
		// random point in hallway = [4, -3, 1]

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
