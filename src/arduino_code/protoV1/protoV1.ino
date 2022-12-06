
/*
    Robot Power MultiMoto v1.0 demo
    This software is released into the Public Domain
    
    This demo shows the setup of the L9958 chips and runs the 4 motor outputs
    independently.  Also Timer0 and Timer1 are set to higher frequency than standard.
Expand
Drivetrain_Direction_Code_Version_3_FINAL.ino
13 KB
﻿
/*
    Robot Power MultiMoto v1.0 demo
    This software is released into the Public Domain
    
    This demo shows the setup of the L9958 chips and runs the 4 motor outputs
    independently.  Also Timer0 and Timer1 are set to higher frequency than standard.
    This may affect the millisecond timer function.
    
    v1.0 - initial release

*/

//This code has all movement protocol defined as functions

// include the SPI library:
#include <SPI.h>
#include <SoftwareSerial.h>
SoftwareSerial BTserial(0, 1); // RX | TX

// inlclude ROS library and MSG type
#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

// include ROS msg type


//#define 1sec 1250

//Number to Object Table
#define WHITE_PEDESTAL 2
#define GREEN_PEDESTAL 3
#define RED_PEDESTAL 4
#define YELLOW_DUCK 5

// L9958 slave select pins for SPI
#define SS_M4 14
#define SS_M3 13
#define SS_M2 12
#define SS_M1 11

// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7

// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
#define PWM_M4 6     // Timer0

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwm1, pwm2, pwm3, pwm4;
int dir1, dir2, dir3, dir4;
int onesecond = 7500;
char recievedChar;
float Speed = 0.5;

void messageCb(const std_msgs::Int8& toggle_msg) {

  //Forward
  if(toggle_msg.data == WHITE_PEDESTAL)
    {
      forward(Speed);
      delay(2000);
      stopRobot();
    }

  //Right
  else if(toggle_msg.data == GREEN_PEDESTAL)
    {
      right(Speed);
      delay(2000);
      stopRobot();
      
    }

  //Backwards
  else if(toggle_msg.data == RED_PEDESTAL)
    {
      backward(Speed);
      delay(2000);
      stopRobot();
    }

  //Left
  else if(toggle_msg.data == YELLOW_DUCK)
    {
      left(Speed);
      delay(2000);
      stopRobot();
    }
  //None of the given 4
    else if(toggle_msg.data == 0)
    {
      stopRobot();
    }

  else {
    stopRobot();
  }
}

ros::Subscriber<std_msgs::Int8> sub("chatter", messageCb);

void setup() {
  
  unsigned int configWord;
  
  // put your setup code here, to run once:
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);  // HIGH = not selected
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);
  
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0
  
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  
  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled
  
  /******* Set up L9958 chips *********
  ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate
  '9 - ISR - current slew rate
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */
  
  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;
  
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high
  
  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH); 
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);
  
  // Reduce the PWM frequency to about 8kHz
  // Note, this will screw up the timer functions that use Timer0 such as millis()
  setPwmFrequency(PWM_M1, 8);
  setPwmFrequency(PWM_M3, 8);

  nh.initNode();
  nh.subscribe(sub);

}

void left(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 1);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 0);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 0);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 1);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void right(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 0);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 1);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 0);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void forward(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 0);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 0);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 0);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 0);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void backward(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 1);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 1);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 1);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void rotateL(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 1);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 0);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 1);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 0);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void rotateR(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 0);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 0);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 1);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void diagonalNE(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 0);
  analogWrite(PWM_M2, 0);  //digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, 0);  //digitalWrite(DIR_M3, 0);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 0);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void diagonalNW(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, 0);  //digitalWrite(DIR_M1, 0);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 0);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 0);
  analogWrite(PWM_M4, 0);  //digitalWrite(DIR_M4, 0);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void diagonalSE(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, speed);  digitalWrite(DIR_M1, 1);
  analogWrite(PWM_M2, 0);  //digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, 0);  //digitalWrite(DIR_M3, 1);
  analogWrite(PWM_M4, speed);  digitalWrite(DIR_M4, 1);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void diagonalSW(float norm){
  int speed = norm * 255;
  
  analogWrite(PWM_M1, 0);  //digitalWrite(DIR_M1, 1);
  analogWrite(PWM_M2, speed);  digitalWrite(DIR_M2, 1);
  analogWrite(PWM_M3, speed);  digitalWrite(DIR_M3, 1);
  analogWrite(PWM_M4, 0);  //digitalWrite(DIR_M4, 1);

  digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
}

void stopRobot(){
  
//  analogWrite(PWM_M1, 0);  digitalWrite(DIR_M1, 1);
//  analogWrite(PWM_M2, 0);  digitalWrite(DIR_M2, 1);
//  analogWrite(PWM_M3, 0);  digitalWrite(DIR_M3, 1);
//  analogWrite(PWM_M4, 0);  digitalWrite(DIR_M4, 1);
  //digitalWrite(ENABLE_MOTORS, LOW);
 // delay (37500);
  digitalWrite(ENABLE_MOTORS, HIGH);  // enable = LOW
}

// *******************************************
// ************** Main Loop ******************
// *******************************************
void loop() {
  nh.spinOnce();
  delay(15);
  
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) { // Timer0 or Timer1
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) { 
      TCCR0B = TCCR0B & 0b11111000 | mode; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode; // Timer1
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode; // Timer2
  }
}
