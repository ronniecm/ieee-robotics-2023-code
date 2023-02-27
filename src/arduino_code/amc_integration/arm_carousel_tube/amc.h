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

#define servoMIN 103
#define servoMAX 512

#define upper_limit 39  //upper limit switch
#define lower_limit 38  //lower limit switch

#define flipMIN 125
#define flipMAX 530

class Amc {
  public:
    Amc();
    ~Amc();
    void activate_paddle();
    void dispense_stack_helper();
    void dispense_helper(int i, int pedestal);
    void dispense();
    void drop_in_action();
    void update_slots(int dir);
    void storeUprightPedestal();
    void liftArm();
    void lowerArm();
    
  private:
    Adafruit_PWMServoDriver* servos;
    Adafruit_PWMServoDriver* steppers;
    Adafruit_TCS34725* tcs;
    char input;
    int deg1;
    int deg2;
    int deg3;
    int deg4;
    int deg5;
    int deg6;
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
