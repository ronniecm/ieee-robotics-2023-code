
#include "ros.h"
#include "std_msgs/Float32.h"
#include "Ultrasonic.h"

//Front Left trig and echo pins and distance variables
#define Front_Left_trigPin 12   // Trigger
#define Front_Left_echoPin 11   // Echo
float duration_Front_Left,inches_Front_Left;

//Back Left trig and echo pins and distance variables
#define Back_Left_trigPin 21
#define Back_Left_echoPin 20
float duration_Back_Left,inches_Back_Left;

//Top Left trig and echo pins and distance variables
#define Top_Left_trigPin 4
#define Top_Left_echoPin 3
float duration_Top_Left,inches_Top_Left;

//Front Right trig and echo pins and distance variables
#define Front_Right_trigPin 23
#define Front_Right_echoPin 22
float duration_Front_Right,inches_Front_Right;

//Back Right trig and echo pins and distance variables
#define Back_Right_trigPin 14
#define Back_Right_echoPin 13
float duration_Back_Right,inches_Back_Right;
//Bottom Right trig and echo pins and distance variables
#define Bottom_Right_trigPin 10
#define Bottom_Right_echoPin 9
float duration_Bottom_Right,inches_Bottom_Right;

float inches, duration;

//ROS TOPICS:
//'/bot/ultraFront'
//'/bot/ultraRight'
//'/bot/ultraBack'
//'/bot/ultraLeft'
//Each topic includes a tuple of the sensor data for each side of the bot


ros::NodeHandle nh;
//Initalize the message type for each topic
std_msgs::Float32 distMsgFrontRight;
std_msgs::Float32 distMsgFrontLeft;
std_msgs::Float32 distMsgTopRight;
std_msgs::Float32 distMsgBottomRight;

std_msgs::Float32 distMsgBackRight;
std_msgs::Float32 distMsgBackLeft;
std_msgs::Float32 distMsgTopLeft;
std_msgs::Float32 distMsgBottomLeft;

//distMsgFrontRight;
//distMsgFrontLeft;
//distMsgTopRight;
//distMsgBottomRight;
//
//distMsgBackRight;
//distMsgBackLeft;
//distMsgTopLeft;
//distMsgBottomLeft;


ros::Publisher frontRight("/ultraFrontRight", &distMsgFrontRight);
ros::Publisher frontLeft("/bot/ultraFrontLeft", &distMsgFrontLeft);
ros::Publisher topRight("/bot/ultraTopRight", &distMsgTopRight);
ros::Publisher bottomRight("/bot/ultraBottomRight", &distMsgBottomRight);

ros::Publisher backRight("/bot/ultraFront", &distMsgBackRight);
ros::Publisher backLeft("/bot/ultraRight", &distMsgBackLeft);
ros::Publisher topLeft("/bot/ultraBack", &distMsgTopLeft);
ros::Publisher bottomLeft("/bot/ultraLeft", &distMsgBottomLeft);

Ultrasonic ultraSensors;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Time to init the sensor node
  nh.initNode();

  //pinmode declarations for ultrasonic sensors
  pinMode(Front_Left_trigPin,OUTPUT);
  pinMode(Front_Left_echoPin,INPUT);
 
  //Back Left
  pinMode(Back_Left_trigPin,OUTPUT);
  pinMode(Back_Left_echoPin,INPUT);

  //Top Left
  pinMode(Top_Left_trigPin,OUTPUT);
  pinMode(Top_Left_echoPin,INPUT);
  
  //Front Right
  pinMode(Front_Right_trigPin,OUTPUT);
  pinMode(Front_Right_echoPin,INPUT);

  //Back Right
  pinMode(Back_Right_trigPin,OUTPUT);
  pinMode(Back_Right_echoPin,INPUT);

  //Bottom Right
  pinMode(Bottom_Right_trigPin,OUTPUT);
  pinMode(Bottom_Right_echoPin,INPUT);
  
 
  //This make the information on the topic available to subscribers
  nh.advertise(frontRight);
  nh.advertise(frontLeft);
  nh.advertise(backRight);
  nh.advertise(backLeft);

  nh.advertise(topRight);
  nh.advertise(topLeft);
  nh.advertise(bottomRight);
  nh.advertise(bottomLeft);

}
 
void loop() {
    //We will save corresponding sensor data to the tuple msg
    
    // --------------------IMPORTANT------------------------
    //TO KEEP THINGS CONSISTANT 0 index ==> LEFT SENSOR & 1 index ==> RIGHT for each side
    
    distMsgFrontLeft.data = ultraSensors.getFrontLeftDistance();
    distMsgFrontRight.data = ultraSensors.getFrontRightDistance();
   
    distMsgTopRight.data = ultraSensors.getTopRightDistance();
    distMsgBottomRight.data = ultraSensors.getBottomRightDistance();

    distMsgBackRight.data = ultraSensors.getBackRightDistance();
    distMsgBackLeft.data = ultraSensors.getBackLeftDistance();
    
    distMsgBottomLeft.data = ultraSensors.getBottomLeftDistance();
    distMsgTopLeft.data = ultraSensors.getTopLeftDistance();

    nh.spinOnce();
    
    frontRight.publish(&distMsgFrontRight);
    frontLeft.publish(&distMsgFrontLeft);
    backRight.publish(&distMsgBackRight);
    backLeft.publish(&distMsgBackLeft);

    topRight.publish(&distMsgTopRight);
    bottomRight.publish(&distMsgBottomRight);
    topLeft.publish(&distMsgTopLeft);
    bottomLeft.publish(&distMsgBottomLeft);
    

}
