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
  for(int i = 2; i < 8; i++) {
    nh.advertise(*ultraSensors.getPub(i));
  }

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
      ultraSensors.publishData();
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
        Serial.println("PROBLEM WITH RIGHT");
      }
      //leftPub.publish(&leftMsg);
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
    Serial.println(F("Failed to boot second VL6180X"));
    //while (1);
  }
  rSensor.setAddress(RIGHT_ADDRESS);
  Serial.println("Done with right");
  delay(10);
  Serial.println("setup complete");
}
