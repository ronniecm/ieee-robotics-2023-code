#include "Drivetrain.h"
#include "Ultraso"
#include "IntervalTimer.h"

#define FL_in1 1
#define FL_in2 2

#define FR_in1 6
#define FR_in2 7

#define BR_in1 5
#define BR_in2 6

#define BL_in1 28
#define BL_in2 29

Drivetrain* drivetrain;
IntervalTimer intTimer;

void setup()
{
    Serial.begin(115200);
    drivetrain = new Drivetrain();
    initDrivetrain();
}

void loop()
{
  // Mecanum drive now a function of twist msgs
 drivetrain->mecanumDrive(0,1.0, 0.0);
 ultrason
 for(int i = 0; i < 4; i++) {
   Serial.print(drivetrain->getRPM(i));
   Serial.print(" ");
 }
 Serial.println();
  //analogWrite(BL_in1, 0);
  //analogWrite(BL_in2, 255);
  
}

void initDrivetrain()
{
  pinMode(FL_in1, OUTPUT);
  pinMode(FL_in2, OUTPUT);
  pinMode(FR_in1, OUTPUT);
  pinMode(FR_in2, OUTPUT);
  pinMode(BL_in1, OUTPUT);
  pinMode(BL_in2, OUTPUT);
  pinMode(BR_in1, OUTPUT);
  pinMode(BR_in2, OUTPUT);

  intTimer.begin(calcRPM, 800);
}

void calcRPM()
{
  drivetrain->calcRPM();
}
