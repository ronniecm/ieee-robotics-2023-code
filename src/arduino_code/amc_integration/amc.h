#ifndef AMC_H
#define AMC_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Encoder.h>
//#include <nRF24L01.h>
//#include <RF24.h>
#include "Adafruit_TCS34725.h"
#include "Arduino.h"





class Amc {
  public:
    Amc();
    ~Amc();
    
    //Variables needed to carousel control
    int stepsLeft = 0;
    int carouselSpeed = 1000;
    int speedControl = 0;
    bool drop_in = false;
    bool dispense_stack = false;
    bool dispensed = false;
    bool dispense_stack_start = false;
    bool dispense_stack_finish = false;
    slots[] = {0,0,0,0,0}; //slot assignment start from tube & ccw: 0,1,2,3,4
    tube[] = {0,0,0}; //What is in tube
    built[] = {0,0,0}; //{0} for 3 stack, [1],[2] for 2 stack
    uint16_t r, g, b, c. colorTemp, lux; 

  

    void activate_paddle();
    void stepperContinue();
    void dispense_stack_helper();
    void dispense_helper(int i, int pedestal);
    void dispense();
    void drop_in_action();
    void update_slots(int dir);
    void storeUprightPedestal(const int msg);
    void liftArm();
    void lowerArm();

    //Functions for all servo controll
    void gripperRotateCmd(int angle);
    void gripperClampCmd(int angle);
    void doorCmd(int angle);
    void armCmd(int angle);
    void wristCmd(int angle);
    void paddleCmd(int angle);
    void liftingCmd(int angle);
    void carouselCmd(int angle);
  

    
  private:
    Adafruit_PWMServoDriver* servos;
    Adafruit_PWMServoDriver* steppers;
    Adafruit_TCS34725* tcs;
    char input;

    int lift;
    int stepsLeft;
    int carouselSpeed;
    int speedControl;
    int slots[5];
    int tube[3];
    int built[3];
    int temp;
    bool drop_in;
    bool dispense_stack;
    bool dispensed;
    bool dispense_stack_start;
    bool dispense_stack_finish;
    int drive_state;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
    uint16_t colorTemp;
    uint16_t lux;
};

#endif
