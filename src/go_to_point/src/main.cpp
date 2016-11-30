//#include "go_to_point.cpp"
#include "interface.cpp"

using namespace std;

// This is where we start
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "go_to_point");

	// Conctruct the class "GoToPoint"
	// GoToPoint goTo;
	Interface inter;

	ros::spin();
	return 0;
}