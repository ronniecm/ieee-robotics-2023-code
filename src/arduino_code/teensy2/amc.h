#ifndef AMC_H
#define AMC_H

#include <Wire.h>
extern TwoWire Wire1;
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
private:
  Adafruit_PWMServoDriver *servos;
  //Adafruit_TCS34725 *tcs;
  //Adafruit_TCS34725 *foodChipTcs;
};

#endif
