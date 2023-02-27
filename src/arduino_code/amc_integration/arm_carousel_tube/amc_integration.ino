#include "ros.h"
#include "std_msgs/Int8.h"
#include "amc.h"
#include "TeensyThreads.h"

ros::NodeHandle nh;
//Initalize the message type for each topic
std_msgs::Int8 gripperRotateMsg;
std_msgs::Int8 gripperClampMsg;
std_msgs::Int8 doorMsg;
std_msgs::Int8 armMsg;
std_msgs::Int8 wristMsg;
std_msgs::Int8 paddleMsg;
std_msgs::Int8 liftingMsg;
std_msgs::Int8 carouselMsg;

ros::Publisher GripperRotate("/bot/gripperRotate_callback", &gripperRotateMsg);
ros::Publisher GripperClamp("/bot/gripperClamp_callback", &gripperClampMsg);
ros::Publisher Door("/bot/door_callback", &doorMsg);
ros::Publisher Arm("/bot/arm_callback", &armMsg);
ros::Publisher Wrist("/bot/wrist_callback", &wristMsg);
ros::Publisher Paddle("/bot/paddle_callback", &paddleMsg);
ros::Publisher Lifting("/bot/lifting_callback", &liftingMsg);
ros::Publisher Carousel("/bot/carousel_callback", &carouselMsg);

void gripperRotateCmd(const std_msgs::Int8& cmd_message)
{
  
}

ros::Subscriber <std_msgs::Int8> sub1("/bot/gripperRotate_cmd", &gripperRotateCmd);

void gripperClampCmd(const std_msgs::Int8& cmd_message)
{
  
}

ros::Subscriber <std_msgs::Int8> sub2("/bot/gripperClamp_cmd", &gripperClampCmd);

void doorCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub3("/bot/door_cmd", &doorCmd);


void armCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub4("/bot/arm_cmd", &armCmd);


void wristCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub5("/bot/wrist_cmd", &wristCmd);


void paddleCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub6("/bot/paddle_cmd", &paddleCmd);


void liftingCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub7("/bot/lifting_cmd", &liftingCmd);


void carouselCmd(const std_msgs::Int8& cmd_message)
{
  
}
ros::Subscriber <std_msgs::Int8> sub8("/bot/carousel_cmd", &carouselCmd);


void setup()
{
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
}

void loop()
{
  gripperRotateMsg.data = 1;
  GripperRotate.publish(&gripperRotateMsg);

  gripperClampMsg.data = 1;
  GripperClamp.publish(&gripperClampMsg);

  doorMsg.data = 1;
  Door.publish(&doorMsg);

  armMsg.data = 1;
  Arm.publish(&armMsg);

  wristMsg.data = 1;
  Wrist.publish(&wristMsg);

  paddleMsg.data = 1;
  Paddle.publish(&paddleMsg);

  liftingMsg.data = 1;
  Lifting.publish(&liftingMsg);

  carouselMsg.data = 1;
  Carousel.publish(&carouselMsg);

  nh.spinOnce();
  delay(100);
  
}
