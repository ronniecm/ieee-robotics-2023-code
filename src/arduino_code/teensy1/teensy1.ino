#include "Drivetrain.h"
#include "IntervalTimer.h"
#include "Ultrasonic.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <Wire.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

Drivetrain* drivetrain;
Ultrasonic* ultrasonics;

IntervalTimer intTimer;

ros::NodeHandle nh;

float cmd_x, cmd_y, cmd_z;

void mecanumCallback(const geometry_msgs::Twist& msg) {
  cmd_x = msg.linear.x;
  cmd_y = msg.linear.y;
  cmd_z = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> speedSub("/bot/cmd_vel", mecanumCallback);

unsigned long currentMillis, previousMillis;

void setup()
{
    //Serial.begin(115200);
    nh.initNode();
    drivetrain = new Drivetrain();
    initDrivetrain();
    ultrasonics = new Ultrasonic();
    initUltrasonics();
    for(int i = 0; i < 6; i++) {
      nh.advertise(*ultrasonics->getPub(i));
    }
    nh.subscribe(speedSub);    
}

void loop()
{
  // Mecanum drive now a function of twist msgs
  currentMillis = millis();
  while (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    drivetrain->mecanumDrive(cmd_y, cmd_x,  cmd_z);
    //drivetrain->mecanumDrive(0.0, 0.80, 0.0);
  }
  ultrasonics->publishData();
  nh.spinOnce();
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
