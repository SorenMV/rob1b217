#ifndef GO_TO_POINT_H
#define GO_TO_POINT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/UInt16.h"
#include "tf/transform_listener.h"

#define db_size 4


// Array with type DBstruct created
struct DBstruct
{
    std::string name; 
    uint16_t key;
    double x, y, z, w;
} db[db_size];

class GoToPoint
{
    public:
        GoToPoint();

    protected:
        // Initialize variables
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
        move_base_msgs::MoveBaseGoal goal;
        ros::Subscriber sub_from_joy;
        ros::NodeHandle nh;

        // Define functions
        void init_db();
        void save_db();
        void go_to_point(const move_base_msgs::MoveBaseGoal&);
        void go_to_db_point(int& i);
        void callback_from_joy_trigger(const std_msgs::UInt16);
        void callback_from_actionlib(const actionlib::SimpleClientGoalState&);

};

#endif // GO_TO_POINT_H