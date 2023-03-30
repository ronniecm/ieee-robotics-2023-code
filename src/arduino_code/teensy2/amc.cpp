//#include <stdint.h>
#include "amc.h"

#define flipMIN 120
#define flipMAX 530

#define wristMIN 120
#define wristMAX 530

#define servoMIN 103
#define servoMAX 512
#define upper_limit 39  //upper limit switch
#define lower_limit 38  //lower limit switch

#define carouselDir 14
#define carouselStep 15

#define elevatorDir 20
#define elevatorStep 21

#define WHITE 1
#define GREEN 2
#define RED 3
#define EMPTY 0

Amc::Amc()
{
  servos = new Adafruit_PWMServoDriver(0x41);
  //tcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
  //foodChipTcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

  servos->begin();
  servos->setPWMFreq(50);

  // color sensor
  //tcs->begin();
  //foodChipTcs->begin(0x29, &Wire1);

  //servos->setPWM(6, 0, map(90, 0, 180, servoMIN, servoMAX));

  // color sensor stuff:
}

Amc::~Amc()
{
  delete servos;
  //delete tcs;
  //delete foodChipTcs;
}

// Functions for all servo controll
void Amc::gripperRotateCmd(int angle)
{

  servos->setPWM(0, 0, map(angle, 0, 180, servoMIN, servoMAX));
}

void Amc::gripperClampCmd(int angle)
{
  servos->setPWM(1, 0, map(angle, 0, 180, servoMIN, servoMAX));
}

void Amc::wristCmd(int angle)
{
  servos->setPWM(2, 0, map(angle, 0, 270, wristMIN, wristMAX));
}

void Amc::doorCmd(int angle)
{
  servos->setPWM(5, 0, map(angle, 0, 180, servoMIN, servoMAX));
}

void Amc::armCmd(int angle)
{
  servos->setPWM(3, 0, map(angle, 0, 180, flipMIN, flipMAX));
}

void Amc::paddleCmd(int angle)
{
  servos->setPWM(4, 0, map(angle, 0, 180, servoMIN, servoMAX));
}

void Amc::foodChipCmd(int angle) {
  servos->setPWM(6, 0, map(angle, 0, 180, servoMIN, servoMAX));  
}
