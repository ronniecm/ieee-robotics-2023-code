#include <Adafruit_VL6180X.h>

#include "ros.h"
#include <Wire.h>
#include "std_msgs/Float32.h"

//extern TwoWire Wire1;

#define SHUTDOWN_LEFT 27
#define SHUTDOWN_RIGHT 28

#define LEFT_ADDRESS 0x30
#define RIGHT_ADDRESS 0x31

Adafruit_VL6180X lSensor = Adafruit_VL6180X();
Adafruit_VL6180X rSensor = Adafruit_VL6180X();

ros::NodeHandle nh;

std_msgs::Float32 leftMsg;
std_msgs::Float32 rightMsg;

ros::Publisher leftPub("/bot/leftTOF", &leftMsg);
ros::Publisher rightPub("/bot/rightTOF", &rightMsg);

void setup() {
  Serial.begin(115200);

  nh.initNode();
  
  pinMode(SHUTDOWN_LEFT, OUTPUT);
  pinMode(SHUTDOWN_RIGHT, OUTPUT);

  digitalWrite(SHUTDOWN_LEFT, LOW);
  digitalWrite(SHUTDOWN_RIGHT, LOW);

  setID();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
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
  //Serial.println("setting up left");
  if (!lSensor.begin()) {
    //Serial.println(F("Failed to boot lSensor VL6180X"));
    while (1);
  }
  Serial.println("Done with left");
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
  Serial.println("Done with right");
  delay(10);
  //Serial.println("setup complete");
}

void loop() {
  
  uint8_t leftStatus = lSensor.readRangeStatus();
  uint8_t rightStatus = rSensor.readRangeStatus();
  float leftVal, rightVal;
  if (leftStatus == VL6180X_ERROR_NONE) {
    leftVal = lSensor.readRange() * 0.1;
    leftMsg.data = leftVal;
    Serial.print("Left: ");
    Serial.print(lSensor.readRange() / 10.0);  
    Serial.println();
  } else {
     //Serial.println("PROBLEM WITH LEFT"); 
   }

   if (rightStatus == VL6180X_ERROR_NONE) {
    rightVal = rSensor.readRange() * 0.1;
    rightMsg.data = rightVal;
    Serial.print("Right: ");
    Serial.print(rSensor.readRange() / 10.0);  
    Serial.println();
  }  else {
    //Serial.println("PROBLEM WITH RIGHT");
  }
  //leftPub.publish(&leftMsg);
  //rightPub.publish(&rightMsg);
  //nh.spinOnce();
  delay(10);
}
