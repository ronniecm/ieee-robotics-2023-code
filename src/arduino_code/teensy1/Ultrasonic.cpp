#include "Ultrasonic.h"
/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be eight ultrasonic sensors around the robot (2 on each side)
and create distance calculations of knowing where the robot is relative to the gameboard
*/

//Ultrasonic Sensor 2 trig and echo pins and distance variables
int duration_Ultra_2,cm_Ultra2;

//Back Left trig and echo pins and distance variables
int duration_Ultra_3,cm_Ultra3;

//Back Left trig and echo pins and distance variables
int duration_Ultra_4,cm_Ultra4;

//Back right trig and echo pins and distance variables
int duration_Ultra_5, cm_Ultra5;

//Back right trig and echo pins and distance variables
int duration_Ultra_6, cm_Ultra6;

//Back right trig and echo pins and distance variables
int duration_Ultra_7, cm_Ultra7;

//code for teensy 3.2 communication
//using corresponding Serial1,2, etc based on RX and TX Value
//Using Serial1 therefore RX1 & TX1 ports

Ultrasonic::Ultrasonic() {
  Serial.println("Constructing");
  ultraPublishers[0] = new ros::Publisher("bot/ultra2", &ultraMessages[0]);
  ultraPublishers[1] = new ros::Publisher("bot/ultra3", &ultraMessages[1]);
  ultraPublishers[2] = new ros::Publisher("/bot/ultra4", &ultraMessages[2]);
  ultraPublishers[3] = new ros::Publisher("/bot/ultra5", &ultraMessages[3]);
  ultraPublishers[4] = new ros::Publisher("/bot/ultra6", &ultraMessages[4]);
  ultraPublishers[5] = new ros::Publisher("/bot/ultra7", &ultraMessages[5]);
  Serial.println("constructed publishers");
  for(int i = 0; i < 6; i++) {
    filters[i] = new MovingAverageFilter(5);
  }
}

Ultrasonic::~Ultrasonic() {
  delete[] filters;
  delete[] ultraPublishers;
}
//for each of the sensor measurements followed reference listed in comment header
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
  //Serial.print(cm_Ultra2);
  //Serial.println(" Cm at Ultrasonic 2 ");
  readings[0] = filters[0]->process(cm_Ultra2);
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
  //Serial.print(cm_Ultra3);
  //Serial.println(" Cm at Ultrasonic 3 ");
  readings[1] = filters[1]->process(cm_Ultra3);
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
  //Serial.print(cm_Ultra4);
  //Serial.println(" Cm at Ultrasonic 4 ");
  readings[2] = filters[2]->process(cm_Ultra4);
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
  //Serial.print(cm_Ultra5);
  //Serial.println(" Cm at Ultrasonic 5 ");
  readings[3] = filters[3]->process(cm_Ultra5);
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
  //Serial.print(cm_Ultra6);
  //Serial.println(" Cm at Ultrasonic 6 ");
  readings[4] = filters[4]->process(cm_Ultra6);
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
  //Serial.print(cm_Ultra7);
  //Serial.println(" Cm at Ultrasonic 7 ");
  readings[5] = filters[5]->process(cm_Ultra7);
  return cm_Ultra7;
}

float Ultrasonic::getReading(int i) {
  return readings[i];  
}

void Ultrasonic::publishData() {
  getUltra2_Distance();
  getUltra3_Distance();
  getUltra4_Distance();
  getUltra5_Distance();
  getUltra6_Distance();
  getUltra7_Distance();
  for(int i = 0; i < 6; i++) {
    ultraMessages[i].data = readings[i];
    //Serial.print(readings[i]);
    //Serial.print(" ");
    ultraPublishers[i]->publish(&ultraMessages[i]);
  }
  //Serial.println();  
}

ros::Publisher* Ultrasonic::getPub(int i) {
  return ultraPublishers[i];
}