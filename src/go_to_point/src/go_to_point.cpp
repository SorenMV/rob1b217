#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <String>


class Route
{
private:
	unsigned int stops_initialized;
	geometry_msgs::PointStamped from, to;
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
		// std::swap(from, to);
		// _send_goal(to);
	}

	
	void _command_send_cb(const String& msg)
	{
		ROS_INFO("Recived: \"%s\" command.", msg);

		const geometry_msgs::PointStamped& to;
		
		// Lookup db to associate a command string with coordinates
		
		// Hard coded coordinates:
		to = {1,0,0,1};
		to.header.frame_id = "/map"; // ? 


		_go_to_point(to);
	}

public:
	Route():client("move_base")
	{
		ros::NodeHandle n;
		marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
		click_sub = n.subscribe("command_send", 1, &Route::_command_send_cb, this);
	};
	~Route(){};
};

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "busroute");

	Route r;

	ros::spin();
	return 0;
}
