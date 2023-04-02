#ifndef AMC_H
#define AMC_H

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <SPI.h>
//#include <Encoder.h>
// #include <nRF24L01.h>
// #include <RF24.h>
#include "Adafruit_TCS34725.h"
#include "Arduino.h"

class Amc
{
public:
  Amc();
  ~Amc();

  // Functions for all servo controll
  void gripperRotateCmd(int angle);
  void gripperClampCmd(int angle);
  void doorCmd(int angle);
  void armCmd(int angle);
  void wristCmd(int angle);
  void paddleCmd(int angle);
  void foodChipCmd(int angle);
  
  //Functions for stepper controll
  void carouselCmd(int stackType);
  void liftingCmd(int dir);
    
  //Sorting Variables

  int stepsLeft = 0;
  int carouselSpeed = 1000;
  int speedControl = 0;
  uint16_t r, g, b, c, lux;
  bool onWhite = true;
  bool onGreen = false;
  bool onRed = false;
  bool onTwo = true;
  bool onThree = false;
  bool drop_in = false;
  bool dispense_stack = false;
  bool dispensed = false;
  bool dispense_stack_start = false;
  bool dispense_stack_finish = false;
  int slots[5] = {0, 0, 0, 0, 0}; // slot assignment start from tube & ccw: 0,1,2,3,4
  int tube[3] = {0, 0, 0};        // What is in tube
  int built[3] = {0, 0, 0};       //{0} for 3 stack, [1],[2] for 2 stack
  
  
  //Sorting Functions
  void fillStack(int stackType);
  void stepperContinue();
  void drop_in_action();
  void dispense();
  void dispense_stack_helper();
  void update_slots(int dir);
  void dispense_helper(int index, int pedestal);
  int findEmptySlot();
  int findPedestalSlot(int pedestal);
  void initSlotFour(int index);
  void activate_paddle();
  void getColorData();

private:
  Adafruit_PWMServoDriver *servos;
  Adafruit_TCS34725 *tcs;
  //Adafruit_TCS34725 *foodChipTcs;
};

#endif
