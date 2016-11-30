#include "go_to_point.cpp"


class Interface : GoToPoint
{

public:
	void checkInput(int input)
	{
		system("clear");
		int locationchoice;

		switch (input)
		{
		 	case (1):
		 	system("canberra-gtk-play -f voice_out/blaster.ogg");
			//Lookup db for key_pressed string and get coordinates. Line by line from the top.
			for (int i = 0; i < db_size; ++i)
			{
				if(db[i].name!="")
				{
					cout<<i+1<<" - "<<db[i].name<<" - "<<db[i].key<<endl;
				}
			}
			cout<<"0 - Back"<<endl;
			cin>>locationchoice;
		   	/* goal.target_pose.header.frame_id = "map";

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
			*/

		}
	}

	
	Interface()
	{
		int choice,location;
		cout << '\a';
	   // system("canberra-gtk-play -f voice_out/welcome.ogg");
		do
		{
			system("clear");
			cout<<"*************************"<<endl;
			cout<<"1 - Locations"<<endl;
			cout<<"1 - Save Location"<<endl;
			cout<<"2 - Controller Settings"<<endl;
			cout<<"3 - Help"<<endl;
			cout<<"9 - EXIT"<<endl;
			cout<<"*************************"<<endl;
		  	//  system("canberra-gtk-play -f voice_out/blaster2.ogg");
			cin>>choice;

			checkInput(choice);
		}
		while(choice!=9);

	}

};
