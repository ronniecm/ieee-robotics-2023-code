#include "ros.h"
#include "amc.h"
#include "TeensyThreads.h"

#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

Amc* arm;

//Initalize the message type for each topic
std_msgs::Int8 gripperRotateMsg;
std_msgs::Int8 gripperClampMsg;
std_msgs::Int8 doorMsg;
std_msgs::Int8 armMsg;
std_msgs::Int8 wristMsg;
std_msgs::Int8 paddleMsg;
std_msgs::Int8 liftingMsg;
std_msgs::Int8 carouselMsg;

//Will init and create default values for each servo

int gripperRotateCmd = 0;
int gripperClampCmd = 180;
int doorCmd = 100;
int armCmd = 180;
int wristCmd = 180;
int paddleCmd = 180;
int liftingCmd = 0;
int carouselCmd = 0;


ros::Publisher GripperRotate("/bot/gripperRotate_callback", &gripperRotateMsg);
ros::Publisher GripperClamp("/bot/gripperClamp_callback", &gripperClampMsg);
ros::Publisher Door("/bot/door_callback", &doorMsg);
ros::Publisher Arm("/bot/arm_callback", &armMsg);
ros::Publisher Wrist("/bot/wrist_callback", &wristMsg);
ros::Publisher Paddle("/bot/paddle_callback", &paddleMsg);
ros::Publisher Lifting("/bot/lifting_callback", &liftingMsg);
ros::Publisher Carousel("/bot/carousel_callback", &carouselMsg);


void gripperRotateCB(const std_msgs::Int8& cmd_msg)
{
  gripperClampCmd = cmd_msg.data;  
  
}

void gripperClampCB(const std_msgs::Int8& cmd_msg)
{
  gripperClampCmd = cmd_msg.data;

}

void doorCB(const std_msgs::Int8& cmd_msg)
{
  doorCmd = cmd_msg.data;
  
}

void armCB(const std_msgs::Int8& cmd_msg)
{
  armCmd = cmd_msg.data;
  
}

void wristCB(const std_msgs::Int8& cmd_msg)
{
  wristCmd = cmd_msg.data;
  
}

void paddleCB(const std_msgs::Int8& cmd_msg)
{
  paddleCmd = cmd_msg.data;
  
}

void liftingCB(const std_msgs::Int8& cmd_msg)
{
  liftingCmd = cmd_msg.data;
}

void carouselCB(const std_msgs::Int8& cmd_msg)
{
  carouselCmd = cmd_msg.data;
}

ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int8> sub1("/bot/gripperRotate_cmd", &gripperRotateCB);
ros::Subscriber <std_msgs::Int8> sub2("/bot/gripperClamp_cmd", &gripperClampCB);
ros::Subscriber <std_msgs::Int8> sub3("/bot/door_cmd", &doorCB);
ros::Subscriber <std_msgs::Int8> sub4("/bot/arm_cmd", &armCB);
ros::Subscriber <std_msgs::Int8> sub5("/bot/wrist_cmd", &wristCB);
ros::Subscriber <std_msgs::Int8> sub6("/bot/paddle_cmd", &paddleCB);
ros::Subscriber <std_msgs::Int8> sub7("/bot/lifting_cmd", &liftingCB);
ros::Subscriber <std_msgs::Int8> sub8("/bot/carousel_cmd", &carouselCB);


void setup()
{

  arm = new Amc;
  nh.initNode();
  nh.advertise(GripperRotate);
  nh.subscribe(sub1);
  nh.advertise(GripperClamp);
  nh.subscribe(sub2);
  nh.advertise(Door);
  nh.subscribe(sub3);
  nh.advertise(Arm);
  nh.subscribe(sub4);
  nh.advertise(Wrist);
  nh.subscribe(sub5);
  nh.advertise(Paddle);
  nh.subscribe(sub6);
  nh.advertise(Lifting);
  nh.subscribe(sub7);
  nh.advertise(Carousel);
  nh.subscribe(sub8);

  pinMode(33, OUTPUT);
  pinMode(38, INPUT);
  pinMode(39, INPUT);
}

void loop()
{
//  gripperRotateMsg.data = 1;
//  GripperRotate.publish(&gripperRotateMsg);
//
//  gripperClampMsg.data = 1;
//  GripperClamp.publish(&gripperClampMsg);
//
//  doorMsg.data = 1;
//  Door.publish(&doorMsg);
//
//  

//
//  wristMsg.data = 1;
//  Wrist.publish(&wristMsg);
//
//  paddleMsg.data = 1;
//  Paddle.publish(&paddleMsg);
//
//  liftingMsg.data = 1;
//  Lifting.publish(&liftingMsg);
//
//  carouselMsg.data = 1;
//  Carousel.publish(&carouselMsg);

  arm->gripperRotateCmd(cmd_msg.data);
  arm->gripperClampCmd(cmd_msg.data);
  arm->doorCmd(cmd_msg.data);
  arm->armCmd(cmd_msg.data);
  arm->wristCmd(cmd_msg.data);
  arm->paddleCmd(cmd_msg.data);
  arm->liftingCmd(cmd_msg.data);
  arm->carouselCmd(cmd_msg.data);


  nh.spinOnce();
  delay(100);
  
}
