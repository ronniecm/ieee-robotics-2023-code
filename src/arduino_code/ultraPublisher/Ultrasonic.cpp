#include "Ultrasonic.h"
/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be eight ultrasonic sensors around the robot (2 on each side)
and create distance calculations of knowing where the robot is relative to the gameboard
*/
//Ultrasonic Sensor 0 trig and echo pins and distance variables
#define Ultra0_trigPin 12   // Trigger
#define Ultra0_echoPin 11   // Echo
float duration_Ultra0, cm_Ultra0;

//Ultrasonic Sensor 1 trig and echo pins and distance variables
#define Ultra1_trigPin 4
#define Ultra1_echoPin 3
float duration_Ultra1, cm_Ultra1;

//Ultrasonic Sensor 2 trig and echo pins and distance variables
#define Ultra2_trigPin 16
#define Ultra2_echoPin 15
float duration_Ultra2, cm_Ultra2;

//Ultrasonic Sensor 3 trig and echo pins and distance variables
#define Ultra3_trigPin 21
#define Ultra3_echoPin 20
float duration_Ultra3, cm_Ultra3;

//Ultrasonic Sensor 4 trig and echo pins and distance variable
#define Ultra4_trigPin 23
#define Ultra4_echoPin 22
float duration_Ultra4, cm_Ultra4;

//Ultrasonic Sensor 5 trig and echo pins and distance variables
#define Ultra5_trigPin 14
#define Ultra5_echoPin 13
float duration_Ultra5, cm_Ultra5;

//Ultrasonic Sensor 6 trig and echo pins and distance variables
#define Ultra6_trigPin 10
#define Ultra6_echoPin 9
float duration_Ultra6, cm_Ultra6;

//Ultrasonic Sensor 7 trig and echo pins and distance variables
#define Ultra7_trigPin 8
#define Ultra7_echoPin 7
float duration_Ultra7, cm_Ultra7;

//code for teensy 3.2 communication
//using corresponding Serial1,2, etc based on RX and TX Value
//Using Serial1 therefore RX1 & TX1 ports


//for each of the sensor measurements followed reference listed in comment header
float Ultrasonic::getUltra0_Distance()
{
  digitalWrite(Ultra0_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra0_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra0_trigPin, LOW);
  //pinMode(Ultra0_echoPin, INPUT);
  duration_Ultra0 = pulseIn(Ultra0_echoPin, HIGH);
  float temp_inches = (duration_Ultra0/2) / 74;   // Divide by 74 or multiply by 0.0135
  //convert inches to cm
  cm_Ultra0 = 2.54 *(temp_inches);
  Serial.print(cm_Ultra0);
  Serial.println(" Cm at Ultrasonic 0 ");
  return cm_Ultra0;

}

float Ultrasonic::getUltra1_Distance()
{
//Clear the sensor
  digitalWrite(Ultra1_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra1_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra1_trigPin, LOW);
  //pinMode(Ultra1_echoPin, INPUT);
  duration_Ultra1 = pulseIn(Ultra1_echoPin, HIGH);
  float temp_inches = (duration_Ultra1/2) / 74;
  cm_Ultra1 = 2.54*(temp_inches);
  Serial.print(cm_Ultra1);
  Serial.println(" Cm at Ultrasonic 1 ");
  return cm_Ultra1;
}

float Ultrasonic::getUltra2_Distance()
{
  digitalWrite(Ultra2_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra2_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra2_trigPin,LOW);
  //pinMode(Ultra2_echoPin, INPUT);
  duration_Ultra2 = pulseIn(Ultra2_echoPin, HIGH);
  float temp_inches = (duration_Ultra2/2) / 74;  
  cm_Ultra2 = 2.54 *(temp_inches);
  Serial.print(cm_Ultra2);
  Serial.println(" Cm at Ultrasonic 2 ");
  return cm_Ultra2;
}

float Ultrasonic::getUltra3_Distance()
{
  digitalWrite(Ultra3_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra3_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra3_trigPin,LOW);
  //pinMode(Ultra3_echoPin, INPUT);
  duration_Ultra3 = pulseIn(Ultra3_echoPin, HIGH);
  float temp_inches = (duration_Ultra3/2) / 74; 
  cm_Ultra3 = 2.54 *(temp_inches);
  Serial.print(cm_Ultra3);
  Serial.println(" Cm at Ultrasonic 3 ");
  return cm_Ultra3;
}

float Ultrasonic::getUltra4_Distance()
{
  digitalWrite(Ultra4_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra4_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra4_trigPin,LOW);
  //pinMode(Ultra4_echoPin, INPUT);
  duration_Ultra4 = pulseIn(Ultra4_echoPin, HIGH);
  float temp_inches = (duration_Ultra4/2) / 74;  
  cm_Ultra4 = 2.54*(temp_inches);
  Serial.print(cm_Ultra4);
  Serial.println(" Cm at Ultrasonic 4 ");
  return cm_Ultra4;
}

float Ultrasonic::getUltra5_Distance()
{
  digitalWrite(Ultra5_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra5_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra5_trigPin,LOW);
  //pinMode(Ultra5_echoPin, INPUT);
  duration_Ultra5 = pulseIn(Ultra5_echoPin, HIGH);
  float temp_inches = (duration_Ultra5/2) / 74;  
  cm_Ultra5 = 2.54*(temp_inches);
  Serial.print(cm_Ultra5);
  Serial.println(" Cm at Ultrasonic 5 ");
  return cm_Ultra5;
}

float Ultrasonic::getUltra6_Distance()
{
  digitalWrite(Ultra6_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra6_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra6_trigPin,LOW);
  //pinMode(Ultra6_echoPin, INPUT);
  duration_Ultra6 = pulseIn(Ultra6_echoPin, HIGH);
  float temp_inches = (duration_Ultra6/2) / 74;  
  cm_Ultra6 = 2.54*(temp_inches);
  Serial.print(cm_Ultra6);
  Serial.println(" Cm at Ultrasonic 6 ");
  return cm_Ultra6;
}

float Ultrasonic::getUltra7_Distance()
{
  digitalWrite(Ultra7_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra7_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra7_trigPin,LOW);
  //pinMode(Ultra7_echoPin, INPUT);
  duration_Ultra7 = pulseIn(Ultra7_echoPin, HIGH);
  float temp_inches = (duration_Ultra7/2) / 74;  
  cm_Ultra7 = 2.54*(temp_inches);
  Serial.print(cm_Ultra7);
  Serial.println(" Cm at Ultrasonic 7 ");
  return cm_Ultra7;
}