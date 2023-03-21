#include "ros.h"
#include "std_msgs/Float32.h"
#include "Ultrasonic.h"
#include "TeensyThreads.h"
#include <Adafruit_VL6180X.h>
#include <Wire.h>

//Ultrasonic Sensor 0 trig and echo pins and distance variables
#define Ultra0_trigPin 3    // Trigger
#define Ultra0_echoPin 2    // Echo


//Ultrasonic Sensor 1 trig and echo pins and distance variables
#define Ultra1_trigPin 4
#define Ultra1_echoPin 5


//Ultrasonic Sensor 2 trig and echo pins and distance variables
#define Ultra2_trigPin 21
#define Ultra2_echoPin 20


//Ultrasonic Sensor 3 trig and echo pins and distance variables
#define Ultra3_trigPin 16
#define Ultra3_echoPin 12


//Ultrasonic Sensor 4 trig and echo pins and distance variable
#define Ultra4_trigPin 14 
#define Ultra4_echoPin 13 


//Ultrasonic Sensor 5 trig and echo pins and distance variables
#define Ultra5_trigPin 11
#define Ultra5_echoPin 10


//Ultrasonic Sensor 6 trig and echo pins and distance variables
#define Ultra6_trigPin 9
#define Ultra6_echoPin 8


//Ultrasonic Sensor 7 trig and echo pins and distance variables
#define Ultra7_trigPin 7
#define Ultra7_echoPin 6



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

//ros::Publisher Ultra0("/bot/ultra0", &distMsgUltra0);
//ros::Publisher Ultra1("/bot/ultra1", &distMsgUltra1);
ros::Publisher Ultra2("/bot/ultra2", &distMsgUltra2);
ros::Publisher Ultra3("/bot/ultra3", &distMsgUltra3);

ros::Publisher Ultra4("/bot/ultra4", &distMsgUltra4);
ros::Publisher Ultra5("/bot/ultra5", &distMsgUltra5);
ros::Publisher Ultra6("/bot/ultra6", &distMsgUltra6);
ros::Publisher Ultra7("/bot/ultra7", &distMsgUltra7);

Ultrasonic ultraSensors;

#define SHUTDOWN_LEFT 27
#define SHUTDOWN_RIGHT 28

#define LEFT_ADDRESS 0x30
#define RIGHT_ADDRESS 0x31

Adafruit_VL6180X lSensor = Adafruit_VL6180X();
Adafruit_VL6180X rSensor = Adafruit_VL6180X();

std_msgs::Float32 leftMsg;
std_msgs::Float32 rightMsg;

ros::Publisher leftPub("/bot/leftTOF", &leftMsg);
ros::Publisher rightPub("/bot/rightTOF", &rightMsg);

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

void setup() {
  //Serial Port begin
  //Time to init the sensor node
  nh.initNode();

  //pinmode declarations for ultrasonic sensors numbering
  //pinMode(Ultra0_trigPin,OUTPUT);
  //pinMode(Ultra0_echoPin,INPUT);
 
  //pinMode(Ultra1_trigPin,OUTPUT);
  //pinMode(Ultra1_echoPin,INPUT);

  pinMode(Ultra2_trigPin,OUTPUT);
  pinMode(Ultra2_echoPin,INPUT);
  
  pinMode(Ultra3_trigPin,OUTPUT);
  pinMode(Ultra3_echoPin,INPUT);

  pinMode(Ultra4_trigPin,OUTPUT);
  pinMode(Ultra4_echoPin,INPUT);

  pinMode(Ultra5_trigPin,OUTPUT);
  pinMode(Ultra5_echoPin,INPUT);

  pinMode(Ultra6_trigPin,OUTPUT);
  pinMode(Ultra6_echoPin,INPUT);

  pinMode(Ultra7_trigPin,OUTPUT);
  pinMode(Ultra7_echoPin,INPUT);
  
 
  //This make the information on the topic available to subscribers
  //nh.advertise(Ultra0);
  //nh.advertise(Ultra1);
  nh.advertise(Ultra2);
  nh.advertise(Ultra3);
  nh.advertise(Ultra4);
  nh.advertise(Ultra5);
  nh.advertise(Ultra6);
  nh.advertise(Ultra7);
  pinMode(SHUTDOWN_LEFT, OUTPUT);
  pinMode(SHUTDOWN_RIGHT, OUTPUT);

  digitalWrite(SHUTDOWN_LEFT, LOW);
  digitalWrite(SHUTDOWN_RIGHT, LOW);

  setID();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
}



void loop() {
    //We will save corresponding sensor data to the tuple msg
    
    // --------------------IMPORTANT------------------------
    //TO KEEP THINGS CONSISTANT 0 index ==> LEFT SENSOR & 1 index ==> RIGHT for each side

      currentMillis = millis();      

      ultraSensors.getUltra2_Distance();
      distMsgUltra2.data = ultraSensors.getReading(2);
      Ultra2.publish(&distMsgUltra2);

      ultraSensors.getUltra3_Distance();
      distMsgUltra3.data = ultraSensors.getReading(3);
      Ultra3.publish(&distMsgUltra3);

      ultraSensors.getUltra4_Distance();
      distMsgUltra4.data = ultraSensors.getReading(4);
      Ultra4.publish(&distMsgUltra4);

      ultraSensors.getUltra5_Distance();
      distMsgUltra5.data = ultraSensors.getReading(5);
      Ultra5.publish(&distMsgUltra5);

      ultraSensors.getUltra6_Distance();
      distMsgUltra6.data = ultraSensors.getReading(6);
      Ultra6.publish(&distMsgUltra6);

      ultraSensors.getUltra7_Distance();
      distMsgUltra7.data = ultraSensors.getReading(7);
      Ultra7.publish(&distMsgUltra7);  

      for(int i = 2; i < 8; i++) {
        Serial.print(ultraSensors.getReading(i));
        Serial.print(" ");  
      }
      Serial.println();
      
      uint8_t leftStatus = lSensor.readRangeStatus();
      uint8_t rightStatus = rSensor.readRangeStatus();
      float leftVal, rightVal;
      /*
      if (leftStatus == VL6180X_ERROR_NONE) {
        leftVal = lSensor.readRange() * 0.1;
        leftMsg.data = leftVal;
        //Serial.print("Left: ");
        //Serial.print(leftVal);
        //Serial.println();
      } else {
        //Serial.println("PROBLEM WITH LEFT"); 
      }
    */
     if (rightStatus == VL6180X_ERROR_NONE) {
        rightVal = rSensor.readRange() * 0.1;
        rightMsg.data = rightVal;
        //Serial.print("Right: ");
        //Serial.print(rSensor.readRange() / 10.0);  
        //Serial.println();
      } else {
        //Serial.println("PROBLEM WITH RIGHT");
      }
      leftPub.publish(&leftMsg);
      rightPub.publish(&rightMsg);
      nh.spinOnce();
      delay(10);
}

void setID() {
  // all reset
  digitalWrite(SHUTDOWN_LEFT, LOW);
  digitalWrite(SHUTDOWN_RIGHT, LOW);

  delay(10);

  // all unreset
  digitalWrite(SHUTDOWN_LEFT, HIGH);
  digitalWrite(SHUTDOWN_RIGHT, HIGH);

  delay(10);

  // activating left sensor and reseting right sensor
  digitalWrite(SHUTDOWN_LEFT, HIGH);
  digitalWrite(SHUTDOWN_RIGHT, LOW);


  // initing Left sensor
  ////Serial.println("setting up left");
  if (!lSensor.begin()) {
    //Serial.println(F("Failed to boot lSensor VL6180X"));
    //while (1);
  }
  //Serial.println("Done wiht left");
  lSensor.setAddress(LEFT_ADDRESS);
  delay(10);

  // activating rSensor
  digitalWrite(SHUTDOWN_RIGHT, HIGH);
  delay(10);

  //initing rSensor
  if (!rSensor.begin(&Wire1)) {
    //Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  rSensor.setAddress(RIGHT_ADDRESS);
  //Serial.println("Done with right");
  delay(10);
  //Serial.println("setup complete");
}
