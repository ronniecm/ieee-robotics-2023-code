#include "Drivetrain.h"
#include "IntervalTimer.h"
#include "Ultrasonic.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <Adafruit_VL6180X.h>
#include <Wire.h>

#define FL_in1 1
#define FL_in2 2

#define FR_in1 6
#define FR_in2 7

#define BR_in1 5
#define BR_in2 6

#define BL_in1 28
#define BL_in2 29

Drivetrain* drivetrain;
Ultrasonic* ultrasonics;
//TOF ROS data stuff
std_msgs::Float32 TOF;
ros::Publisher TOFPub("/bot/tof", &TOF);
Adafruit_VL6180X vl = Adafruit_VL6180X();

IntervalTimer intTimer;

ros::NodeHandle nh;

float cmd_x, cmd_y, cmd_z;

void mecanumCallback(const geometry_msgs::Twist& msg) {
  cmd_x = msg.linear.x;
  cmd_y = msg.linear.y;
  cmd_z = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> speedSub("/bot/cmd_vel", mecanumCallback);

void setup()
{
    Serial.begin(115200);
    nh.initNode();
    Serial.println("initialized node");
    drivetrain = new Drivetrain();
    initDrivetrain();
    Serial.println("constructed drivetrain");
    ultrasonics = new Ultrasonic();
    initUltrasonics();
    nh.advertise(TOFPub);
    Serial.println("constructed ultrasonics");
    for(int i = 0; i < 6; i++) {
      nh.advertise(*ultrasonics->getPub(i));
    }
    nh.subscribe(speedSub);
    Serial.println("advertised and subscribed to topics");
     //Below sets up the TOF
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  else
  {
    Serial.println("Sensor found");
  }
}

void loop()
{
  // Mecanum drive now a function of twist msgs
  drivetrain->mecanumDrive(cmd_y, cmd_x,  cmd_z);
  ultrasonics->publishData();
  //analogWrite(BL_in1, 0);
  //analogWrite(BL_in2, 255);
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  float range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  if (status == VL6180X_ERROR_NONE) {
    Serial.println("Getting value of TOF");
    float to_cm = range * 0.1;
    Serial.println(to_cm);
    //Serial.println(lux);
    TOF.data = to_cm;
    
  }
  else
  {
    Serial.println("TOF sucks RONNIE");
  }
  TOFPub.publish(&TOF);
  nh.spinOnce();
  delay(10);
}

void initDrivetrain()
{
  pinMode(FL_in1, OUTPUT);
  pinMode(FL_in2, OUTPUT);
  pinMode(FR_in1, OUTPUT);
  pinMode(FR_in2, OUTPUT);
  pinMode(BL_in1, OUTPUT);
  pinMode(BL_in2, OUTPUT);
  pinMode(BR_in1, OUTPUT);
  pinMode(BR_in2, OUTPUT);

  intTimer.begin(calcRPM, 800);
}

void calcRPM()
{
  drivetrain->calcRPM();
}

void initUltrasonics() {
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
}
