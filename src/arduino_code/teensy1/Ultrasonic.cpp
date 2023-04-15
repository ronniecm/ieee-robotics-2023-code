#include "core_pins.h"
#include "Ultrasonic.h"
/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be eight ultrasonic sensors around the robot (2 on each side)
and create distance calculations of knowing where the robot is relative to the gameboard
*/

Ultrasonic::Ultrasonic() {
  ultraPublishers[0] = new ros::Publisher("bot/ultra2", &ultraMessages[0]);
  ultraPublishers[1] = new ros::Publisher("bot/ultra3", &ultraMessages[1]);
  ultraPublishers[2] = new ros::Publisher("/bot/ultra4", &ultraMessages[2]);
  ultraPublishers[3] = new ros::Publisher("/bot/ultra5", &ultraMessages[3]);
  ultraPublishers[4] = new ros::Publisher("/bot/ultra6", &ultraMessages[4]);
  ultraPublishers[5] = new ros::Publisher("/bot/ultra7", &ultraMessages[5]);
  Serial.println("constructed publishers");
  for(int i = 0; i < 6; i++) {
    filters[i] = new MovingAverageFilter(4);
    sonars[i] = new NewPing(trigPins[i], echoPins[i], 400);
  }
}

Ultrasonic::~Ultrasonic() {
  delete[] filters;
  delete[] ultraPublishers;
  delete[] sonars;
}

float Ultrasonic::getDistance(int i) {
  /*
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(5);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
  float duration = pulseIn(echoPins[i], HIGH);
  float inches = (duration / 2) / 74;
  float cm = 2.54 * inches;
  readings[i] = filters[i]->process(cm);
  //readings[i] = cm;
  return cm;  
  */
  readings[i] = sonars[i]->ping_cm(400);
  return readings[i];
}

float Ultrasonic::getReading(int i) {
  return readings[i];  
}

void Ultrasonic::publishData() {
  for(int i = 0; i < 6; i++) {
    getDistance(i);
    if (readings[i] != 0) {
      ultraMessages[i].data = readings[i];
      ultraPublishers[i]->publish(&ultraMessages[i]);
    }
  }
  delay(5);
}

ros::Publisher* Ultrasonic::getPub(int i) {
  return ultraPublishers[i];
}
