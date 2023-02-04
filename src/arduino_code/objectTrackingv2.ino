//#define USE_TEENSY_HW_SERIAL

// inlclude ROS library and MSG type
#include "ros.h"
#include <std_msgs/Int8.h>
#include <util/atomic.h>

ros::NodeHandle nh;

// include ROS msg type


//#define 1sec 1250

//Number to Object Table
#define FORWARD 0
#define SEARCH 1
#define RIGHT 2
#define LEFT 3
#define STOP 4
#define ROTATE 5
#define REVERSE 6

#define ENA_1 0
#define ENB_1 5
#define ENA_2 6
#define ENB_2 11

#define IN1_1 1
#define IN2_1 2
#define IN3_1 3
#define IN4_1 4

#define IN1_2 7
#define IN2_2 8
#define IN3_2 9
#define IN4_2 10

#define ENCA_1 27 
#define ENCB_1 28

#define ENCA_2 31
#define ENCB_2 32

#define ENCA_3 37
#define ENCB_3 38

#define ENCA_4 33
#define ENCB_4 34

#define ENCPOWER 39

int counter = 0;
float Speed = 0.85;

volatile int pos1_i = 0;
volatile int pos2_i = 0;
volatile int pos3_i = 0;
volatile int pos4_i = 0;

void messageCb(const std_msgs::Int8& toggle_msg) {

  //Forward
  if(toggle_msg.data == SEARCH)
    {
      counter++;
      if(counter <= 50) {
        forward(Speed); 
      }
      else {
        rotateR(Speed);  
      }
      if (counter > 100) {
        counter = 0;
      }
    }

  //Right
  else if(toggle_msg.data == RIGHT)
    {
      right(Speed); 
    }

  //Left
  else if(toggle_msg.data == LEFT)
    {
      left(Speed);
    }
  //None of the given 4
    else if(toggle_msg.data == STOP)
    {
      stopRobot();
    }
    else if(toggle_msg.data == FORWARD)
    {
      forward(Speed); 
    }
    else if (toggle_msg.data == ROTATE)
    { 
      rotateL(.7);
    }
    else if (toggle_msg.data == REVERSE)
    { 
      reverse(Speed);
    }
  else {
    stopRobot();
  }
}

ros::Subscriber<std_msgs::Int8> sub("chatter", messageCb);

void setup() {
  // Serial.begin(9600);
  pinMode(ENA_1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(ENB_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  
  pinMode(ENCA_1, INPUT);
  pinMode(ENCB_1, INPUT);
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_2, INPUT);

  pinMode(ENA_2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(ENB_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  
  pinMode(ENCA_3, INPUT);
  pinMode(ENCB_3, INPUT);
  pinMode(ENCA_4, INPUT);
  pinMode(ENCB_4, INPUT);

  pinMode(ENCPOWER, OUTPUT);
  digitalWrite(ENCPOWER, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCA_1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_4), readEncoder4, RISING);


  //nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  
}

// *******************************************
// ************** Main Loop ******************
// *******************************************
void loop() {
  int pos1 = 0;
  int pos2 = 0;
  int pos3 = 0;
  int pos4 = 0;
  //float velocity = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = pos1_i;
    pos2 = pos2_i;
    pos3 = pos3_i;
    pos4 = pos4_i;
    //velocity = velocity_i;
  }
  nh.spinOnce();
  delay(15);
}

void forward(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed);
}

void reverse(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed);
}

void left(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed);  
}

void right(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed);  
}

void rotateL(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed);    
}

void rotateR(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed); 
}

void stopRobot() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, LOW);
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, LOW);
}  

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB_1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos1_i = pos1_i + increment;

  // Compute velocity with method 2
  /*
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
  */
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB_2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos2_i = pos2_i + increment;

  /*
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
  */
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_3);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos3_i = pos3_i + increment;

  /*
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
  */
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_4);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos4_i = pos4_i + increment;

  /*
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
  */
}
