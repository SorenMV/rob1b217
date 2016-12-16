#include "include/go_to_point.h"

using namespace std;

class Interface : GoToPoint
{

public:

	void outputDBChoice(int &i)
	{
		cout << i+1 << " - Name: " << db[i].name << ", Joystick: " << db[i].key << "\n";
	}
	void outputDBChoices()
	{
		for (int i = 0; i < db_size; ++i)
		{
			// Show the entry if EITHER the name or key is set
			if(db[i].name != "" || db[i].key)
			{
				outputDBChoice(i);
			}
		}
	}

	void showLocations()
	{
		int locationchoice;

	 	system("canberra-gtk-play -f voice_out/blaster.ogg");
	 	do
	 	{
	 		system("clear");
		 	cout << "Press a key to go to location or 0 to go back.\n";
			//Lookup db for key_pressed string and get coordinates. Line by line from the top.
			
			outputDBChoices();
			cout << "0 - Back" << "\n";
			
			cin >> locationchoice;
		   	// Check input
			for (int i = 0; i < db_size; ++i)
			{
				// If the user input is in db, go there
				if(i+1 == locationchoice)
				{
					go_to_db_point(i);
					return;
				}
			}
	 	}while(locationchoice != 0);
	}

	void showChangeName()
	{
		int choice;

		do
		{
			system("clear");
			cout << "Press a key to change name of location or 0 to go back.\n";

			outputDBChoices();
			cout << "0 - Back" << "\n";

			cin >> choice;
			
			// Check input
			for (int i = 0; i < db_size; ++i)
			{
				// If the user input IS in db
				if(i+1 == choice)
				{
					outputDBChoice(i);
					cout << "Type the new name: ";
					cin >> db[i].name;
					save_db();
					return;
				}
			}
		}while(choice != 0);

	}

	// Start interface
	void start()
	{
		int choice;
		cout << '\a';
	   	// system("canberra-gtk-play -f voice_out/welcome.ogg");
		do
		{
			system("clear");
			cout << "*************************" << "\n"
				 << "1 - Locations" << "\n"
				 << "2 - Save Location" << "\n"
				 << "3 - Change name on location" << "\n"
				 << "4 - Controller Settings" << "\n"
				 << "9 - Help" << "\n"
				 << "0 - EXIT" << "\n"
				 << "*************************" << "\n";
		  	//  system("canberra-gtk-play -f voice_out/blaster2.ogg");
			cin >> choice;

			switch(choice)
			{
			 	case(1):
			 		showLocations();
			 		break;
			 	// case(2):
			 		// showSaveLocation();
			 	case(3):
			 		showChangeName();
			 		break;
			}
			
		}
		while(choice != 0 && ros::ok());
	}

	// Constructor
	Interface():
		GoToPoint() // run go_to_point Constructor
	{
		start();
	}
	

};
