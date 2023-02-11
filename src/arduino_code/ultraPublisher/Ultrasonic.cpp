#include "Ultrasonic.h"
/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be six ultrasonic sensors around the robot (2 on the left side, 2 on the right side, one on front, one on back)
and create distance calculations of knowing where the robot is relative to the gameboard using a coordinate system
*/

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

//Top Left trig and echo pins and distance variables
#define Top_Right_trigPin 16
#define Top_Right_echoPin 15
float duration_Top_Right,inches_Top_Right;

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

//Bottom Left trig and echo pins and distance variables
#define Bottom_Left_trigPin 8
#define Bottom_Left_echoPin 7
float duration_Bottom_Left,inches_Bottom_Left;

//code for teensy 3.2 communication
//using corresponding Serial1,2, etc based on RX and TX Value
//Using Serial1 therefore RX1 & TX1 ports


//for each of the sensor measurements followed reference listed in comment header
float Ultrasonic::getFrontLeftDistance()
{
  digitalWrite(Front_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Left_trigPin, LOW);
  //pinMode(Front_Left_echoPin, INPUT);
  duration_Front_Left = pulseIn(Front_Left_echoPin, HIGH);
  // Convert the time into a distance
  inches_Front_Left = (duration_Front_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
  Serial.print(inches_Front_Left);
  Serial.println(" Inches at Front Left Sensor");

  return inches_Front_Left;

}

float Ultrasonic::getBackLeftDistance()
{
//below is back left calculations
  digitalWrite(Back_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Left_trigPin, LOW);
  //pinMode(Back_Left_echoPin, INPUT);
  duration_Back_Left = pulseIn(Back_Left_echoPin, HIGH);
  inches_Back_Left = (duration_Back_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
  Serial.print(inches_Back_Left);
  Serial.println(" Inches at Back Left Sensor");

  return inches_Back_Left;
}

float Ultrasonic::getTopLeftDistance()
{
 //below is Top Left calculations
  digitalWrite(Top_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Left_trigPin,LOW);
  //pinMode(Top_Left_echoPin, INPUT);
  duration_Top_Left = pulseIn(Top_Left_echoPin, HIGH);
  inches_Top_Left = (duration_Top_Left/2) / 74;  
  Serial.print(inches_Top_Left);
  Serial.println(" Inches at Top Left Sensor");

  return inches_Top_Left;
}

float Ultrasonic::getTopRightDistance()
{
 //below is Top Right calculations
  digitalWrite(Top_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Right_trigPin,LOW);
  //pinMode(Top_Left_echoPin, INPUT);
  duration_Top_Left = pulseIn(Top_Right_echoPin, HIGH);
  inches_Top_Right = (duration_Top_Right/2) / 74;  
  Serial.print(inches_Top_Right);
  Serial.println(" Inches at Top Left Sensor");

  return inches_Top_Right;
}

float Ultrasonic::getFrontRightDistance()
{
  digitalWrite(Front_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Right_trigPin,LOW);
  //pinMode(Front_Right_echoPin, INPUT);
  duration_Front_Right = pulseIn(Front_Right_echoPin, HIGH);
  inches_Front_Right = (duration_Front_Right/2) / 74;  
  Serial.print(inches_Front_Right);
  Serial.println(" Inches at Front Right Sensor");
 
  return inches_Front_Right;
}

float Ultrasonic::getBackRightDistance()
{
  digitalWrite(Back_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Right_trigPin,LOW);
  //pinMode(Back_Right_echoPin, INPUT);
  duration_Back_Right = pulseIn(Back_Right_echoPin, HIGH);
  inches_Back_Right = (duration_Back_Right/2) / 74;  
  Serial.print(inches_Back_Right);
  Serial.println(" Inches at Back Right Sensor");

  return inches_Back_Right;
}

float Ultrasonic::getBottomRightDistance()
{
  digitalWrite(Bottom_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Bottom_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Bottom_Right_trigPin,LOW);
  //pinMode(Bottom_Right_echoPin, INPUT);
  duration_Bottom_Right = pulseIn(Bottom_Right_echoPin, HIGH);
  inches_Bottom_Right = (duration_Bottom_Right/2) / 74;  
  Serial.print(inches_Bottom_Right);
  Serial.println(" Inches at Bottom Right Sensor");

  return inches_Bottom_Right;
}

float Ultrasonic::getBottomLeftDistance()
{
  digitalWrite(Bottom_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Bottom_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Bottom_Left_trigPin,LOW);
  //pinMode(Bottom_Left_echoPin, INPUT);
  duration_Bottom_Left = pulseIn(Bottom_Left_echoPin, HIGH);
  inches_Bottom_Left = (duration_Bottom_Left/2) / 74;  
  Serial.print(inches_Bottom_Left);
  Serial.println(" Inches at Bottom Right Sensor");

  return inches_Bottom_Left;
}
