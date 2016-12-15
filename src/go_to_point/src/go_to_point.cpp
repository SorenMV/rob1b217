#include <ros/ros.h>
#include <fstream> //data file
#include <actionlib/client/simple_action_client.h> //set of functions; we send goal through it
#include <move_base_msgs/MoveBaseAction.h> //message containing goal coordinated
#include "std_msgs/UInt16.h" //message used to subscribe from joystick_control
#include "tf/transform_listener.h" //get current location


#define db_size 4 //number of saved locations

using namespace std;
	

class GoToPoint
{
private:
	// Initialise variables
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
		double x, y, z, w; //x,y for position ; z,w for orientation
	};

	// Array with type DBstruct created
	struct DBstruct db[db_size];

	// This function gets called when a coordinate is set and wants to move
	int _go_to_point(const move_base_msgs::MoveBaseGoal& goal)
	{
		// Wait for the action server to come up
		int timer=1;
		while( (!(client.waitForServer(ros::Duration(1.0)))) && (timer <= 3))
		{
			ROS_INFO("%i: Waiting for the move_base action server to come up.", timer);
			++timer;
			// Check if ros is ok. If NOT ok, then return
			if(!ros::ok())
				return 0;
		}

		// Send the goal to actionlib and return to the "callback" function
		if(timer<3)
		{
		client.sendGoal(goal, boost::bind(&GoToPoint::_target_reached, this, _1));
		ROS_INFO("Navigating ...");
		return 1;
		}
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
			// This can be because of obstacles making it impossible to get to goal.
			ROS_INFO("Failed to reach target.");
		}
	}

	void _init_db()
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
				ROS_INFO("Goal(%s, %i): (%f, %f, %f, %f)", db[i].name.c_str(),db[i].key, db[i].x, db[i].y, db[i].z, db[i].w);
				
				i++;
			}
			inputFile.close();
			ROS_INFO("DB Initialized.");
		}
	}

	// This function gets called when a key is pressed

	void _callback_from_joy(const std_msgs::UInt16 subscribed_key)
	{
		
		//Lookup db for subscribed_key and get coordinates. Line by line from the top. 
		ROS_INFO("%i", subscribed_key.data);
		// go to point
		if(subscribed_key.data<=3)
		{
		for (int i = 0; i < db_size; ++i)
		{
			if(i == subscribed_key.data)
			{	
				//Feeding coordinations to goal from the specific array in the db array:
				goal.target_pose.pose.position.x = db[i].x;
				goal.target_pose.pose.position.y = db[i].y;
				goal.target_pose.pose.orientation.z = db[i].z;
				goal.target_pose.pose.orientation.w = db[i].w;

				//output the coordinates and the name of the location
				ROS_INFO("Going to: %s(%i): (%f, %f, %f, %f)", db[i].name.c_str(), db[i].key, db[i].x, db[i].y, db[i].z, db[i].w);

				// Send coordinates to next function
				_go_to_point(goal);
			}
		}
		}


		//Save location
		if(subscribed_key.data>=4 && subscribed_key.data<=7)  
		{
			//get current pose:
			try 
				{
		   		tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
					tf_listener.lookupTransform("/map", "/base_link", ros::Time(), tf_stamped);
				}

			catch 
				(
					tf::TransformException ex) {
				   ROS_ERROR("%s",ex.what());
				
				}

		for (int i = 0; i < db_size; ++i)
		{
			if(i == (subscribed_key.data-4))
			{
				//put current coordinates ino array
				db[i].x = tf_stamped.getOrigin().x();
				db[i].y = tf_stamped.getOrigin().y();
				db[i].z = tf_stamped.getRotation().z();
				db[i].w = tf_stamped.getRotation().w();
				ROS_INFO("Location saved on line %i (%f, %f, %f, %f)",i, db[i].x, db[i].y, db[i].z, db[i].w);
			}
		}
		ofstream inputFile("database/location_database.txt");
		if(inputFile.is_open())
		{
			for (int i = 0; i < db_size; ++i)
			{
				//save new coordinates
				inputFile << db[i].name <<" "
					<< i <<" " 
					<< db[i].x <<" " 
					<< db[i].y <<" " 
					<< db[i].z <<" " 
					<< db[i].w <<"\n";
			}
			inputFile.close();
		}
		}

		//cancels goal when joystick is moved / POWER button is pressed
		if(subscribed_key.data==8)
			{
				client.cancelGoal(); 
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
		subscribtion_from_joy = go_to_point_nodehandle.subscribe<std_msgs::UInt16>("go_to_point_trigger", 10, &GoToPoint::_callback_from_joy_to_send_goal, this); 
		
		ROS_INFO("Started. Listening for commands ...");


	}
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point");

	// Conctruct the class "GoToPoint"
	GoToPoint goTo;

	ros::spin();//loop until closed
	return 0;
}
