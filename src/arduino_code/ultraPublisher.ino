
#include "ros.h"
#include "std_msgs/Float64.h"


int trigPin = 12;    // Trigger
int echoPin = 13;    // Echo


float inches, duration;



ros::NodeHandle nh;
std_msgs::Float64 distMsg;
ros::Publisher ultra("/bot/ultraFront", &distMsg);



void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Time to init the sensor node
  nh.initNode();
  nh.advertise(ultra);

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 
void loop() {
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
 
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
  
    duration = pulseIn(echoPin, HIGH);
 
    // Convert the time into a distance
    //cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
    distMsg.data = inches;
    
    ultra.publish(&distMsg);
    nh.spinOnce();
    delay(100);
    //Serial.print(inches);
    //Serial.print("in FRONT, ");
    //Serial.println();

}
