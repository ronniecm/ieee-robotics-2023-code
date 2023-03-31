#include "Drivetrain.h"
#include "Ultrasonic.h"
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
  currentMillis = millis();
  while (currentMillis - previousMillis >= 10)
  {
    previousMillis = currentMillis;
    if (Serial.available() > 0)
    {
      int n = Serial.parseInt();
      demand = (double)n / 10.0;
      // Mecanum drive now a function of twist msgs
      drivetrain->mecanumDrive(0.0, demand, 0.0);
      for (int i = 0; i < 4; i++)
      {
        Serial.print(drivetrain->getRPM(i));
        Serial.print(" ");
      }
      Serial.println();
    }
    
    arm->gripperRotateCmd(gripperRotateCmd.data);
    //GripperRotate.publish(&gripperRotateCmd);


    arm->gripperClampCmd(gripperClampCmd.data);
    //GripperClamp.publish(&gripperClampCmd);

    arm->doorCmd(doorCmd.data);
    //Door.publish(&doorCmd);

    arm->armCmd(armCmd.data);
    //Arm.publish(&armCmd);


    arm->wristCmd(wristCmd.data);
    //Wrist.publish(&wristCmd);


    arm->paddleCmd(paddleCmd.data);
    //Paddle.publish(&paddleCmd);

    //arm->foodChipCmd(foodChipColorCmd.data);
    //foodChip.publish(foodChipColorCmd.data);

    arm->liftingCmd(liftingCmd.data);
    
    if (liftingCmd.data == 1 && digitalRead(UPPER_LIMIT)== LOW) {liftingCmd.data = 0;}
    if (liftingCmd.data == -1 && digitalRead(LOWER_LIMIT)== LOW) {liftingCmd.data = 0;}


    arm->liftingCmd(liftingCmd.data);
    
    arm->carouselCmd(carouselCmd.data);
    //if(arm->getStepsLeft() == 0) {
     //carouselCmd.data = 0;
      //arm->carouselCmd(carouselCmd.data);
    //}

   }
   carouselCmd.data = 0;

   pedestalColorMsg.data[0] = arm->r;
   pedestalColorMsg.data[1] = arm->g;
   pedestalColorMsg.data[2] = arm->b;
   pedestalColorMsg.data[3] = arm->c;
    
   for(int i = 0; i < 5; i++){
    carouselMsg.data[i] = int32_t(arm->slots[i]);
   }

    Carousel.publish(&carouselMsg);

    PedestalColor.publish(&pedestalColorMsg);



    if (foodChipColorCmd.data == 1)
    {
      foodChipColorMsg.data = "detecting";
      foodChip.publish(&foodChipColorMsg);
      foodChipColorCmd.data = 0;
      int color = arm->getFoodChipColor();
      if (color == 0)
      {
        foodChipColorMsg.data = "detected red";
      }
      else
      {
        foodChipColorMsg.data = "detected green";
      }
      foodChip.publish(&foodChipColorMsg);
      arm->foodChipCmd(90);
    }

    delay(10);
    nh.spinOnce();
  }


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
