#include <ros/ros.h>
//getting joy input
#include <sensor_msgs/Joy.h>
//moving the turtlebot
#include <geometry_msgs/Twist.h>
//publishing to go_to_point node
#include "std_msgs/UInt16.h"



//play with values if you dare!
const float deadman_radius           = 0.15     ;

const float linear_velocity          = 0.5      ;
const float angular_velocity         = 1.5      ;

const float period                   = 0.005    ;
const float speed_constant           = 0.0025   ;
const float deceleration_multiplier  = 10       ;





class joystick_class
{
public:
  ros::NodeHandle       joy_nodehandle                     ;
  ros::Subscriber       joy_subscriber                     ;
  ros::Publisher        joy_velocity_publisher             ;
  ros::Publisher        joy_go_to_point_publisher          ;
  ros::Timer            joy_publish_timer                  ;

  //declaring publishing variables
  geometry_msgs::Twist  joy_velocity_published_value       ;
  std_msgs::UInt16      joy_go_to_point_published_number   ;

  //is button pressed? variables
  bool Apressed, Bpressed, Xpressed, Ypressed, LBpressed, RBpressed, backpressed, startpressed, powerpressed, LJpressed, RJpressed;

  //checks whether the kobuki base is turning
  float current_linear_velocity, current_angular_velocity, desired_linear_velocity, desired_angular_velocity;


  //contructor
  joystick_class()
  {
    //"joy" is the topic we are subscribed to
    //"mobile_base/commands/velocity" is a topic controlling the movement of the mobile base
    //"this" is used for passing a reference to the instance of the object (in this case it is the class itself)
    joy_subscriber = joy_nodehandle.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_class::joystick_callback, this); 
    joy_velocity_publisher = joy_nodehandle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

    //publising our own topic "go_to_point_trigger" to trigger the "go_to_point" node
    joy_go_to_point_publisher = joy_nodehandle.advertise<std_msgs::UInt16>("go_to_point_trigger", 1);


    //periodically publishing
    joy_publish_timer = joy_nodehandle.createTimer(ros::Duration(period),  &joystick_class::publishing, this);
  }


private:

  void publishing(const ros::TimerEvent&)
  {
    //stopping while moving forward
    if (desired_linear_velocity == 0 && current_linear_velocity != 0 && current_linear_velocity > 0 )
    {
      if (current_linear_velocity <= deceleration_multiplier*speed_constant)
          {
            current_linear_velocity=0;
          }
      else
      {
        current_linear_velocity=current_linear_velocity-speed_constant;
      }
    }
    else

    //stopping while reversing
    if (desired_linear_velocity == 0 && current_linear_velocity != 0 && current_linear_velocity < 0 )
    {
      if (current_linear_velocity >= (-deceleration_multiplier)*speed_constant)
        {
          current_linear_velocity=0;
        }
      else
        {
          current_linear_velocity=current_linear_velocity+speed_constant;
        }
    }
    else


    if (current_linear_velocity < desired_linear_velocity)
    {
      current_linear_velocity = current_linear_velocity + speed_constant;
    }
    else


    if (current_linear_velocity > desired_linear_velocity)
    {
      current_linear_velocity = current_linear_velocity - speed_constant;
    }
   
      



    //angular velocity
    if (desired_angular_velocity !=0)
    {
      current_angular_velocity = desired_angular_velocity;
    }
    else
    if (desired_angular_velocity ==0)
    {
      current_angular_velocity = 0;
    }


    //publish
    joy_velocity_published_value.linear.x   = current_linear_velocity * linear_velocity;
    joy_velocity_published_value.angular.z  = current_angular_velocity * angular_velocity;
    joy_velocity_publisher.publish(joy_velocity_published_value);
    if(current_linear_velocity==0 && desired_linear_velocity==0)
    {
       joy_publish_timer.stop();
    }
  }


  //callback function executed each time "sensor_msgs/Joy" node publishes something we are subscribed to
  void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
  {

    //JOYSTICK:

    if (joy->axes[0] > deadman_radius || joy->axes[0] < -deadman_radius || joy->axes[1] > deadman_radius || joy->axes[1] < -deadman_radius) //outside
    {
      desired_angular_velocity  = joy->axes[0];
      desired_linear_velocity   = joy->axes[1];

      //cancelGoal
      joy_go_to_point_published_number.data = 8;  
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);

      joy_publish_timer.start();
    }


    if(joy->axes[0] <= deadman_radius && joy->axes[0] >= -deadman_radius && joy->axes[1] <= deadman_radius && joy->axes[1] >= -deadman_radius) //inside
    {
      desired_angular_velocity  = 0;
      desired_linear_velocity   = 0;
    }


  //BUTTONS:

  //emergency stop + cancelGoal 
  //BACK button
    if(joy->buttons[6] == 1)
    {
      joy_velocity_published_value.angular.z = 0;
      joy_velocity_published_value.linear.x = 0;
      joy_velocity_publisher.publish(joy_velocity_published_value);
      //sending cancelGoal request
      joy_go_to_point_published_number.data = 8;  
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
      backpressed = true;
    }
    if(joy->buttons[6] == 0){backpressed = false;}

    //Location buttons
    //RB
    if(joy->buttons[5] == 0)
    {
      //A
      if(Apressed == false && joy->buttons[0] == 1)
      {
      joy_go_to_point_published_number.data = 0;
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
      Apressed = true;
      }
      if(Apressed == true && joy->buttons[0] == 0){Apressed = false;}


      //B
      if(Bpressed == false && joy->buttons[1] == 1)
      {
        joy_go_to_point_published_number.data = 1;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Bpressed = true;
      }
      if(Bpressed == true && joy->buttons[1] == 0){Bpressed = false;}


      //X
      if(Xpressed == false && joy->buttons[2] == 1)
      {
        joy_go_to_point_published_number.data = 2;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Xpressed = true;
      }
      if(Xpressed == true && joy->buttons[2] == 0){Xpressed = false;}


      //Y
      if(Ypressed == false && joy->buttons[3] == 1)
      {
        joy_go_to_point_published_number.data = 3;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Ypressed = true;
      }
      if(Ypressed == true && joy->buttons[3] == 0){Ypressed = false;}
    }


    //Combination of 2 buttons to save location
    //RB
    else if(joy->buttons[5] == 1)
    {
      //A+RB
      if(Apressed == false && joy->buttons[0] == 1)
      {
        joy_go_to_point_published_number.data = 4;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Apressed = true;
      }
      if(Apressed == true && joy->buttons[0] == 0){Apressed = false;}


      //B+RB
      if(Bpressed == false && joy->buttons[1] == 1)
      {
        joy_go_to_point_published_number.data = 5;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Bpressed = true;
      }
      if(Bpressed == true && joy->buttons[1] == 0){Bpressed = false;}


      //X+RB
      if(Xpressed == false && joy->buttons[2] == 1)
      {
        joy_go_to_point_published_number.data = 6;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Xpressed = true;
      }
      if(Xpressed == true && joy->buttons[2] == 0){Xpressed = false;}


      //Y+RB
      if(Ypressed == false && joy->buttons[3] == 1)
      {
        joy_go_to_point_published_number.data = 7;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Ypressed = true;
      }
      if(Ypressed == true && joy->buttons[3] == 0){Ypressed = false;}
    }


    /*
    //LB
    if(LBpressed == false && joy->buttons[4] == 1)
    {
      LBpressed = true;
    }
    if(LBpressed = true && joy->buttons[4] == 0){LBpressed = false;}


    //START button
    if(startpressed == false && joy->buttons[7] == 1)
    { 
      startpressed = true;
    }
    if(startpressed == true && joy->buttons[7] == 0){startpressed = false;}


    //POWER button
    if(powerpressed == false && joy->buttons[8] == 1)
    {   
      powerpressed = true;
    }
    if(powerpressed == true && joy->buttons[8] == 0){powerpressed = false;}

    //LJ
    if(LJpressed == false && joy->buttons[9] == 1)
    {   
      LJpressed = true;
    }
    if(LJpressed == true && joy->buttons[9] == 0){LJpressed = false;}


    //RJ
    if(RJpressed == false && joy->buttons[10] == 1)
    {
      RJpress ed = true;
    }
    if(RJpressed == true && joy->buttons[10] == 0){RJpressed = false;}
    */
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");

// Conctruct the class "joystick_class"
  joystick_class Candy;

// Repeat receiving subscribtion, thus executing callback function
  ros::spin();
}