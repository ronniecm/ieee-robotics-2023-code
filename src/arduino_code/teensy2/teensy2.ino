#include "amc.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/String.h"



#define UPPER_LIMIT 39
#define LOWER_LIMIT 38

//Setting up the ros node for wheel commands
// Setting up the ros node for wheel commands
ros::NodeHandle nh;
//Thie type of message is geometry_msgs::Twist
// Thie type of message is geometry_msgs::Twist



Amc* arm;



//std_msgs::Int32MultiArray carouselMsg;
//std_msgs::UInt16MultiArray pedestalColorMsg;

std_msgs::Int16 gripperRotateCmd;
std_msgs::Int16 gripperClampCmd;
std_msgs::Int16 wristCmd;
std_msgs::Int16 doorCmd;
std_msgs::Int16 paddleCmd;
std_msgs::Int16 armCmd;
std_msgs::Int16 foodChipColorCmd;


void gripperRotateCB(const std_msgs::Int16 &cmd_msg)
{
  gripperRotateCmd.data = cmd_msg.data;
}

void gripperClampCB(const std_msgs::Int16 &cmd_msg)
{
  gripperClampCmd.data = cmd_msg.data;
}


void doorCB(const std_msgs::Int16 &cmd_msg)
{
  doorCmd.data = cmd_msg.data;
}

void armCB(const std_msgs::Int16 &cmd_msg)
{
  armCmd.data = cmd_msg.data;
}

void wristCB(const std_msgs::Int16 &cmd_msg)
{
  wristCmd.data = cmd_msg.data;
}


void paddleCB(const std_msgs::Int16 &cmd_msg)
{
  paddleCmd.data = cmd_msg.data;
}

//This will be the callback function for wheel commands from jetson

void foodChipColorCB(const std_msgs::Int16 &cmd_msg)
{
  foodChipColorCmd.data = cmd_msg.data;
}

ros::Subscriber<std_msgs::Int16> gripperRotate("/bot/gripperRotate_cmd", gripperRotateCB);
ros::Subscriber<std_msgs::Int16> gripperClamp("/bot/gripperClamp_cmd", gripperClampCB);
ros::Subscriber<std_msgs::Int16> door("/bot/door_cmd", doorCB);
ros::Subscriber<std_msgs::Int16> elbow("/bot/arm_cmd", armCB);
ros::Subscriber<std_msgs::Int16> wrist("/bot/wrist_cmd", wristCB);
ros::Subscriber<std_msgs::Int16> paddle("/bot/paddle_cmd", paddleCB);
ros::Subscriber<std_msgs::Int16> foodChipColor("/bot/foodChipColor", foodChipColorCB);

// This will be the callback function for wheel commands from jetson


void setup()
{
    arm = new Amc();

    nh.initNode();
    
    nh.subscribe(gripperRotate);
    nh.subscribe(gripperClamp);
    nh.subscribe(door);
    nh.subscribe(elbow);
    nh.subscribe(wrist);
    nh.subscribe(paddle);
    nh.subscribe(foodChipColor);
    
    gripperRotateCmd.data = 90;
    gripperClampCmd.data = 0;
    wristCmd.data = 180;
    armCmd.data = 180;
    paddleCmd.data = 135;
    doorCmd.data = 120;
    foodChipColorCmd.data = 90;
}

void loop()
{

    arm->gripperRotateCmd(gripperRotateCmd.data);
    

    arm->doorCmd(doorCmd.data);
    

    arm->armCmd(armCmd.data);
   

    arm->paddleCmd(paddleCmd.data);
    

    arm->gripperClampCmd(gripperClampCmd.data);

    arm->wristCmd(wristCmd.data);

    arm->foodChipCmd(foodChipColorCmd.data);
    
    nh.spinOnce();
    delay(10);
}
