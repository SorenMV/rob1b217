#include "go_to_point.cpp"


class Interface : GoToPoint
{

public:
	void showLocations()
	{
		system("clear");
		int locationchoice;

		
	 	system("canberra-gtk-play -f voice_out/blaster.ogg");
		//Lookup db for key_pressed string and get coordinates. Line by line from the top.
		for (int i = 0; i < db_size; ++i)
		{
			if(db[i].name != "")
			{
				cout<<i+1<<" - "<<db[i].name<<" - "<<db[i].key<<endl;
			}
		}
		cout<<"0 - Back"<<endl;
		cin>>locationchoice;
	   	

		/*int i = 0;
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

	// Constructor
	Interface()
	{
		int choice;
		cout << '\a';
	   // system("canberra-gtk-play -f voice_out/welcome.ogg");
		do
		{
			system("clear");
			cout << "*************************" << "\n";
			cout << "1 - Locations" << "\n";
			cout << "2 - Save Location" << "\n";
			cout << "3 - Controller Settings" << "\n";
			cout << "9 - Help" << "\n";
			cout << "0 - EXIT" << "\n";
			cout << "*************************" << "\n";
		  	//  system("canberra-gtk-play -f voice_out/blaster2.ogg");
			cin >> choice;

			switch (choice)
			{
			 	case (1):
			 		showLocations();
			 	
			}
			
		}
		while(choice != 0);

	}

};
