#include <ros/ros.h>
//we need "<sensor_msgs/Joy.h>" to get the "sensor_msgs::Joy" message out of "joy" topic,
//which contains input from joystick: float "axes[8]", int "buttons[11]" 
//and "header" (the time the data is received from the joystick)
#include <sensor_msgs/Joy.h>
//we need this one to send the "geometry_msgs::Twist" message to "mobile_base/commands/velocity" topic (Kobuki),
//which consists of: geometry_msgs/Vector3 "linear" , geometry_msgs/Vector3 "angular";
//"geometry_msgs/Vector3" has float "x", float "y" and float "z"
#include <geometry_msgs/Twist.h>
//we need this one to see if bumper sensor is activated
#include <kobuki_msgs/BumperEvent.h>
//we need this one to honk the horn
#include <kobuki_msgs/Sound.h>
//publishing "std_msgs::UInt16" to go_to_point node
//which contains: uint16_t "data"
#include "std_msgs/UInt16.h"



//distance the joystick gimbal has be offset to allow the manual steering
const float deadman_radius           = 0.15     ;//0-1; 0=always active, 1=never active

//maximum velocity; influences acceleration if above the maximum velocity of the Kobuki
      float linear_velocity          = 0.5      ;//0.5 = 0.44m/s
const float angular_velocity         = 1.5      ;//this value is adequate

//refresh rate of the "joy_publish_timer" timer publishing "geometry_msgs::Twist" message to "mobile_base/commands/velocity" topic
const float period                   = 0.02     ;//50Hz

//directly influeces acceleration/deceleration
const float speed_constant           = 0.01     ;//(+/-0.01% each iteration); from 0 to max speed in 2 seconds if "linear_velocity" does not exceed maximum velocity of the Kobuki

//makes deceleration faster
const float deceleration_multiplier  = 1        ;//1 to be same as acceleration, >1 to decelerate faster



class joystick_class
{
public:
  //VARIABLES:
  ros::NodeHandle       joy_nodehandle                     ;
  ros::Subscriber       joy_subscriber                     ;//subscriber from "joy" (joystick)
  ros::Subscriber  	bumper_subscriber			     ;//subscriber from "mobile_base/events/bumper" (bumper)
  ros::Publisher        joy_velocity_publisher             ;//publisher to "mobile_base/commands/velocity" (Kobuki)
  ros::Publisher        joy_go_to_point_publisher          ;//publisher to "go_to_point"
  ros::Publisher 		honk_publisher			     ;//publisher to "mobile_base/commands/sound" (horn)
  ros::Timer            joy_publish_timer                  ;//timer to continuously publish to "mobile_base/commands/velocity" (Kobuki)  

  //message variables
  geometry_msgs::Twist  joy_velocity_published_value       ;//publishing this to "mobile_base/commands/velocity" (Kobuki)
  std_msgs::UInt16      joy_go_to_point_published_number   ;//publishing this to "go_to_point"
  kobuki_msgs::Sound    honk  				     ;//publishing this to "mobile_base/commands/sound" (horn)

  //is button pressed? variables
  bool Apressed, Bpressed, Xpressed, Ypressed, backpressed, startpressed, emergency_activated;

  //speed smoothing variables
  float current_linear_velocity, desired_linear_velocity, desired_angular_velocity;



  //CONSTRUCTOR:
  joystick_class()
  {
    //"joy" is the joystick input topic
    //"mobile_base/commands/velocity" is a topic controlling the movement of the mobile base (Kobuki)
    joy_subscriber = joy_nodehandle.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_class::joystick_callback, this); 
    bumper_subscriber = joy_nodehandle.subscribe<kobuki_msgs::BumperEvent>("mobile_base/events/bumper", 10, &joystick_class::bumper_callback, this); 
 
    joy_velocity_publisher = joy_nodehandle.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
    honk_publisher = joy_nodehandle.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
    //publising our own topic "go_to_point_trigger" to control the "go_to_point" node
    joy_go_to_point_publisher = joy_nodehandle.advertise<std_msgs::UInt16>("go_to_point_trigger", 1);
    
    //periodically publishing "mobile_base/commands/velocity" to make the movement smooth
    joy_publish_timer = joy_nodehandle.createTimer(ros::Duration(period),  &joystick_class::publishing, this);
  }


private:
  //function called 1/"period" times per second
  void publishing(const ros::TimerEvent&)
  {
  //SMOOTHING
    //smooth stop
    if(desired_linear_velocity == 0)
    { //while moving forward
      if (current_linear_velocity > 0)
      { //if current velocity is close to be 0
        if (current_linear_velocity <= speed_constant)
        {
          current_linear_velocity = 0;
          joy_publish_timer.stop();//stop publishing
        }
        else
        { //otherwise subtract speed
          current_linear_velocity = current_linear_velocity - speed_constant * deceleration_multiplier;
        }
      }

      else
      //while reversing
      if (current_linear_velocity < 0)
      { //if current velocity is close to be 0
        if (current_linear_velocity >= (-1)*speed_constant)
        {
          current_linear_velocity = 0;
          joy_publish_timer.stop();//stop publishing
        }
        else
        { //otherwise add speed
          current_linear_velocity = current_linear_velocity + speed_constant * deceleration_multiplier;
        }
      }

    }

    else 
    //add to match the joystick
    if (current_linear_velocity < desired_linear_velocity)
    {
      current_linear_velocity = current_linear_velocity + speed_constant;
    }

    else
    //subtract to match the joystick
    if (current_linear_velocity > desired_linear_velocity)
    {
      current_linear_velocity = current_linear_velocity - speed_constant;
    }
  //END OF SMOOTHING


    //multiply with maximum speed constants
    joy_velocity_published_value.linear.x   = current_linear_velocity * linear_velocity;
    joy_velocity_published_value.angular.z  = desired_angular_velocity * angular_velocity; //current angular = desired angular; no smoothing
    //publish to "mobile_base/commands/velocity" (Kobuki)
    joy_velocity_publisher.publish(joy_velocity_published_value);

    if (current_linear_velocity == 0 && desired_linear_velocity == 0 && desired_angular_velocity == 0)
    {
   	  joy_publish_timer.stop(); //stop timer to increase efficiency
    }
  }


  void bumper_callback(const kobuki_msgs::BumperEvent bump)
  {
  	if (bump.state == 1) //if bumper sensor is activated
  	{
  	  emergency_activated = true;
  	  joy_publish_timer.stop();//stop publishing, thus moving
        current_linear_velocity=0;//reset current linear velocity
  	}
  }


  //callback function executed each time "sensor_msgs/Joy" topic updates: joystick movements & autorefresh_rate(if set)
  void joystick_callback(const sensor_msgs::Joy joy)
  {
    //emergency stop + cancelGoal 
    if(joy.buttons[8] == 1) //POWER button
    {
      joy_publish_timer.stop();//stop publishing, thus moving
      current_linear_velocity=0;//reset current linear velocity
  
      //stop
      joy_velocity_published_value.angular.z = 0;
      joy_velocity_published_value.linear.x = 0;
      joy_velocity_publisher.publish(joy_velocity_published_value);

      //sending cancelGoal request
      joy_go_to_point_published_number.data = 8;  
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
      emergency_activated = true;//return to default joystick position after emergency break before moving
    }

        
    //MANUAL STEERING:
    //if gimbal is pushed
    if ((emergency_activated == false) && (joy.axes[0] > deadman_radius || joy.axes[0] < -deadman_radius || joy.axes[1] > deadman_radius || joy.axes[1] < -deadman_radius))
    {
      //set desired velocities to mach gimbal values
      desired_angular_velocity  = joy.axes[0];
      desired_linear_velocity   = joy.axes[1];

      //cancelGoal
      joy_go_to_point_published_number.data = 8;  
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);

      //start timer (start smoothing & publishing movement)
      joy_publish_timer.start();
    }

    //if gimbal is in centre
    if(joy.axes[0] <= deadman_radius && joy.axes[0] >= -deadman_radius && joy.axes[1] <= deadman_radius && joy.axes[1] >= -deadman_radius)
    {
      //set desired velocities to 0  
      desired_angular_velocity  = 0;
      desired_linear_velocity   = 0;
      emergency_activated = false;//return to this default joystick position after emergency break before moving
    }


    //BUTTONS:
    //Location buttons
    if(joy.buttons[5] == 0) //RB
    {
      if(Apressed == false && joy.buttons[0] == 1) //A
      {
      joy_go_to_point_published_number.data = 0;
      joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
      Apressed = true;
      }
      if(Apressed == true && joy.buttons[0] == 0){Apressed = false;}


      if(Bpressed == false && joy.buttons[1] == 1) //B
      {
        joy_go_to_point_published_number.data = 1;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Bpressed = true;
      }
      if(Bpressed == true && joy.buttons[1] == 0){Bpressed = false;}


      if(Xpressed == false && joy.buttons[2] == 1) //X
      {
        joy_go_to_point_published_number.data = 2;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Xpressed = true;
      }
      if(Xpressed == true && joy.buttons[2] == 0){Xpressed = false;}


      if(Ypressed == false && joy.buttons[3] == 1) //Y
      {
        joy_go_to_point_published_number.data = 3;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Ypressed = true;
      }
      if(Ypressed == true && joy.buttons[3] == 0){Ypressed = false;}
    }


    //Combination of 2 buttons to save location
    else if(joy.buttons[5] == 1) //RB
    {
      if(Apressed == false && joy.buttons[0] == 1) //A+RB
      {
        joy_go_to_point_published_number.data = 4;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Apressed = true;
      }
      if(Apressed == true && joy.buttons[0] == 0){Apressed = false;}


      if(Bpressed == false && joy.buttons[1] == 1) //B+RB
      {
        joy_go_to_point_published_number.data = 5;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Bpressed = true;
      }
      if(Bpressed == true && joy.buttons[1] == 0){Bpressed = false;}


      if(Xpressed == false && joy.buttons[2] == 1) //X+RB
      {
        joy_go_to_point_published_number.data = 6;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Xpressed = true;
      }
      if(Xpressed == true && joy.buttons[2] == 0){Xpressed = false;}


      if(Ypressed == false && joy.buttons[3] == 1) //Y+RB
      {
        joy_go_to_point_published_number.data = 7;
        joy_go_to_point_publisher.publish(joy_go_to_point_published_number);
        Ypressed = true;
      }
      if(Ypressed == true && joy.buttons[3] == 0){Ypressed = false;}
    }

    //Decrease speed
    if(backpressed == false && joy.buttons[6] == 1) //BACK button
    {   
      if(linear_velocity>0.05)
      {
      	linear_velocity = linear_velocity - 0.05;  
      }   
      backpressed = true;
    }
    if(backpressed == true && joy.buttons[6] == 0){backpressed = false;}

    //Increase speed
    if(startpressed == false && joy.buttons[7] == 1) //START button
    { 
      if(linear_velocity<0.5)
      {
      	linear_velocity = linear_velocity + 0.05; 
      }
      startpressed = true;
    } 
    if(startpressed == true && joy.buttons[7] == 0){startpressed = false;}



    if(joy.buttons[4] == 1) //LB
    {
   	honk.value=1;
      honk_publisher.publish(honk);
    }
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");

// Conctruct the class "joystick_class"
  joystick_class Chukwa_joy;

// Repeat receiving subscribtion, thus executing callback function
  ros::spin();
}



/*
Table of index number of /joy.buttons: 
A                               0
B                               1
X                               2
Y                               3
LB                              4
RB                              5
back                            6
start                           7
power                           8
Button stick left               9
Button stick right              10

Table of index number of /joy.axis: 
Left/Right Axis stick left      0
Up/Down Axis stick left         1
LT                              2
Left/Right Axis stick right     3
Up/Down Axis stick right        4
RT                              5
cross key left/right            6
cross key up/down               7
*/
