#include "ros.h"
#include "std_msgs/Float32.h"
#include "Ultrasonic.h"

//Front Left trig and echo pins and distance variables
#define Front_Left_trigPin 12   // Trigger
#define Front_Left_echoPin 11   // Echo

//Back Left trig and echo pins and distance variables
#define Back_Left_trigPin 21
#define Back_Left_echoPin 20

//Top Left trig and echo pins and distance variables
#define Top_Left_trigPin 4
#define Top_Left_echoPin 3

//Top Right trig and echo pins and distance variables
#define Top_Right_trigPin 16
#define Top_Right_echoPin 15

//Front Right trig and echo pins and distance variables
#define Front_Right_trigPin 23
#define Front_Right_echoPin 22


//Back Right trig and echo pins and distance variables
#define Back_Right_trigPin 14
#define Back_Right_echoPin 13

 
//Bottom Right trig and echo pins and distance variables
#define Bottom_Right_trigPin 10
#define Bottom_Right_echoPin 9

//Bottom Left trig and echo pins and distance variables
#define Bottom_Left_trigPin 8
#define Bottom_Left_echoPin 7



//ROS TOPICS:
//'/bot/ultraFront'
//'/bot/ultraRight'
//'/bot/ultraBack'
//'/bot/ultraLeft'
//Each topic includes a tuple of the sensor data for each side of the bot


ros::NodeHandle nh;
//Initalize the message type for each topic
std_msgs::Float32 distMsgUltra0;
std_msgs::Float32 distMsgUltra1;
std_msgs::Float32 distMsgUltra2;
std_msgs::Float32 distMsgUltra3;

std_msgs::Float32 distMsgUltra4;
std_msgs::Float32 distMsgUltra5;
std_msgs::Float32 distMsgUltra6;
std_msgs::Float32 distMsgUltra7;

//distMsgUltra0;
//distMsgUltra1;
//distMsgUltra2;
//distMsgUltra3;
//
//distMsgUltra4;
//distMsgUltra5;
//distMsgUltra6;
//distMsgUltra7;


ros::Publisher Ultra0("/bot/ultra0", &distMsgUltra0);
ros::Publisher Ultra1("/bot/ultra1", &distMsgUltra1);
ros::Publisher Ultra2("/bot/ultra2", &distMsgUltra2);
ros::Publisher Ultra3("/bot/ultra3", &distMsgUltra3);

ros::Publisher Ultra4("/bot/ultra4", &distMsgUltra4);
ros::Publisher Ultra5("/bot/ultra5", &distMsgUltra5);
ros::Publisher Ultra6("/bot/ultra6", &distMsgUltra6);
ros::Publisher Ultra7("/bot/ultra7", &distMsgUltra7);

Ultrasonic ultraSensors;

void setup() {
  //Serial Port begin
  Serial.begin(9600);
  //Time to init the sensor node
  nh.initNode();

  //pinmode declarations for ultrasonic sensors numbering
  pinMode(Ultra0_trigPin,OUTPUT);
  pinMode(Ultra0_echoPin,INPUT);
 
  pinMode(Ultra1_trigPin,OUTPUT);
  pinMode(Ultra1_echoPin,INPUT);

  pinMode(Ultra2_trigPin,OUTPUT);
  pinMode(Ultra2_echoPin,INPUT);
  
  pinMode(Ultra3_trigPin,OUTPUT);
  pinMode(Ultra3_echoPin,INPUT);

  pinMode(Ultra4_trigPin,OUTPUT);
  pinMode(Ultra4_trigPin,INPUT);

  pinMode(Ultra5_trigPin,OUTPUT);
  pinMode(Ultra5_echoPin,INPUT);

  pinMode(Ultra6_trigPin,OUTPUT);
  pinMode(Ultra6_echoPin,INPUT);

  pinMode(Ultra7_trigPin,OUTPUT);
  pinMode(Ultra7_echoPin,INPUT);
  
 
  //This make the information on the topic available to subscribers
  nh.advertise(Ultra0);
  nh.advertise(Ultra1);
  nh.advertise(Ultra2);
  nh.advertise(Ultra3);
  nh.advertise(Ultra4);
  nh.advertise(Ultra5);
  nh.advertise(Ultra6);
  nh.advertise(Ultra7);

}
 
void loop() {
    //We will save corresponding sensor data to the tuple msg
    
    // --------------------IMPORTANT------------------------
    //TO KEEP THINGS CONSISTANT 0 index ==> LEFT SENSOR & 1 index ==> RIGHT for each side
    
    distMsgUltra0.data = ultraSensors.getUltra0_Distance();
    distMsgUltra1.data = ultraSensors.getUltra1_Distance();
   
    distMsgUltra2.data = ultraSensors.getUltra2_Distance();
    distMsgUltra3.data = ultraSensors.getUltra3_Distance();

    distMsgUltra4.data = ultraSensors.getUltra4_Distance();
    distMsgUltra5.data = ultraSensors.getUltra5_Distance();
    
    distMsgUltra6.data = ultraSensors.getUltra6_Distance();
    distMsgUltra7.data = ultraSensors.getUltra7_Distance();

    nh.spinOnce();
    
    Ultra0.publish(&distMsgUltra0);
    Ultra1.publish(&distMsgUltra1);
    Ultra2.publish(&distMsgUltra2);
    Ultra3.publish(&distMsgUltra3);

    Ultra4.publish(&distMsgUltra4);
    Ultra5.publish(&distMsgUltra5);
    Ultra6.publish(&distMsgUltra6);
    Ultra7.publish(&distMsgUltra7);
    

}