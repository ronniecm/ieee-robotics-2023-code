#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725softi2c.h"

class Dispensing
{
  public:
      void forward(int mspeed);
      void backward(int mspeed);
      void left(int mspeed);
      void right(int mspeed);
      void halt();
      bool checkcolor();
      void setup_bot();
};