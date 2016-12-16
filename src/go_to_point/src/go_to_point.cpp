#include "include/go_to_point.h"

using namespace std;

// This function gets called when a coordinate is set and wants to move
void GoToPoint::go_to_point(const move_base_msgs::MoveBaseGoal& goal)
{
	// Wait for the action server to come up
	int timer = 0;
	while( (!(client.waitForServer(ros::Duration(2.0)))) && (timer <= 3))
	{
		ROS_INFO("Waiting for the move_base action server to come up (%i)", timer);
		++timer;
		// Check if ros is ok. If NOT ok, then return
		if(!ros::ok())
			return;
	}

	if(timer >> 3)
	{
		ROS_INFO("Failed to connect to move_base action server");
	}	

	// Send the goal to actionlib and return to the "callback" function
	client.sendGoal(goal, boost::bind(&GoToPoint::callback_from_actionlib, this, _1));
	ROS_INFO("Navigating ...");
}

// This function gets called when actionlib is done navigating to the goal
void GoToPoint::callback_from_actionlib(const actionlib::SimpleClientGoalState& state)
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

void GoToPoint::init_db()
{
	//Calling the database and naming it inputFile.
	ifstream inputFile("database/location_database.txt");
	string line;

	//Making sure it has succesfully been opened.
	if(inputFile.is_open())
	{	
		int i = 0;

		// Inputting data from the database into the array. 
		// !inputFile.eof() = while NOT end of file. 
		while(!inputFile.eof() && i < db_size)
		{
			// getline() means that it reads the entire line and puts it into secound parameter.
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
		ROS_INFO("DB Initialized");
	}
	else
	{
		ROS_INFO("Failed DB Init");
	}
}

// This function gets called when a key is pressed
void GoToPoint::callback_from_joy_trigger(const std_msgs::UInt16 subscribed_key)
{
	//Lookup db for subscribed_key and get coordinates. Line by line from the top. 
	ROS_INFO("%i", subscribed_key.data);
	if(subscribed_key.data<=3)
	{
		for (int i = 0; i < db_size; ++i)
		{
			if(i == subscribed_key.data)
			{	
				go_to_db_point(i);
			}
		}
	}


	//Save location
	int save_key_offset = 4;
	if(subscribed_key.data>=4 && subscribed_key.data<=7)  
	{
		tf::TransformListener tf_listener;
		tf::StampedTransform tf_stamped;
		try 
		{
   			tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
			tf_listener.lookupTransform("/map", "/base_link", ros::Time(), tf_stamped);
		}
		catch (tf::TransformException ex) 
		{
		   	ROS_ERROR("TransformException: %s",ex.what());
		}


		for(int i = 0; i < db_size; ++i)
		{
			if(i == (subscribed_key.data-4))
			{
				db[i].x = tf_stamped.getOrigin().x();
				db[i].y = tf_stamped.getOrigin().y();
				db[i].z = tf_stamped.getRotation().z();
				db[i].w = tf_stamped.getRotation().w();
				ROS_INFO("Location saved on line %i (%f, %f, %f, %f)", 
					i, db[i].x, db[i].y, db[i].z, db[i].w);
			}
		}


		// Save db
		save_db();
	}

	//cancels goal when joystick is moved / back button is pressed
	if(subscribed_key.data == 8)
	{
		client.cancelGoal();
	}
}

void GoToPoint::go_to_db_point(int& i)
{
	//Feeding coordinations to goal from the specific array in the db array:
	goal.target_pose.pose.position.x = db[i].x;
	goal.target_pose.pose.position.y = db[i].y;
	goal.target_pose.pose.orientation.z = db[i].z;
	goal.target_pose.pose.orientation.w = db[i].w;

	//"cout" the coordinates and the name of the location
	ROS_INFO("Going to: %s(%i): (%f, %f, %f, %f)", 
		db[i].name.c_str(), db[i].key, db[i].x, db[i].y, db[i].z, db[i].w);

	// Send coordinates to next function
	go_to_point(goal);
}

void GoToPoint::save_db()
{
	ofstream inputFile("database/location_database.txt");
	if(inputFile.is_open())
	{
		for(int i = 0; i < db_size; ++i)
		{
			inputFile << db[i].name <<" "
				<< i <<" " 
				<< db[i].x <<" " 
				<< db[i].y <<" " 
				<< db[i].z <<" " 
				<< db[i].w <<"\n";
		}
		inputFile.close();
	}
	else
	{
		ROS_INFO("Failed to save database.");
	}
}


// Constructor
GoToPoint::GoToPoint():
	client("move_base")
{
	init_db();

	// coordinate frame. "map" or "base_link"
	goal.target_pose.header.frame_id = "map";

	// Subscribing to our joystick topic
	sub_from_joy = nh.subscribe<std_msgs::UInt16>("go_to_point_trigger", 10, &GoToPoint::callback_from_joy_trigger, this); 
	
	ROS_INFO("Started. Listening for commands ...");
}
