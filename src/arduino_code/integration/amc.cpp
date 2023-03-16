#include "amc.h"

#define flipMIN 120
#define flipMAX 530
#define servoMIN 103
#define servoMAX 512
#define upper_limit 39  //upper limit switch
#define lower_limit 38  //lower limit switch
#define CarouselPin 33

Amc::Amc() {    
    servos = new Adafruit_PWMServoDriver(0x40);
    steppers = new Adafruit_PWMServoDriver(0x41);
    tcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

    servos->begin();
    servos->setPWMFreq(50);
  
    steppers->begin();
    steppers->setPWMFreq(1000);
    
    if (tcs->begin()) {
    //Serial.println("Found sensor");
    }

    deg1 = 0; //pedestal rotate
    deg2 = 0; //clamp close
    deg3 = 180; //clamp pan
    deg4 = 180; //arm flip
    deg5 = 180; //paddle
    deg6 = 100; // door
   
    stepsLeft = 0;
    carouselSpeed = 1000;
    speedControl = 0;
    drop_in = false;
    dispense_stack = false;
    dispensed = true;
    dispense_stack_start = false;
    dispense_stack_finish = false;
    drive_state = 1; //0: stop; 1: forward; 2:backward
}

Amc::~Amc() {
    delete servos;
    delete steppers;
    delete tcs;
}

//Functions for all servo controll
void Amc::gripperRotateCmd(int angle)
{

  servos->setPWM(0, 0, map( angle, 0, 180, servoMIN, servoMAX));
}

void Amc::gripperClampCmd(int angle)
{
  servos->setPWM(1, 0, map( angle, 0, 180, servoMIN, servoMAX));
}

void Amc::wristCmd(int angle)
{
  servos->setPWM(2, 0, map( angle, 0, 180, servoMIN, servoMAX));
}

void Amc::doorCmd(int angle)
{
  servos->setPWM(5, 0, map( angle, 0, 180, servoMIN, servoMAX));
}

void Amc::armCmd(int angle)
{
  servos->setPWM(3, 0, map( angle, 0, 180, flipMIN, flipMAX));
}

void Amc::paddleCmd(int angle)
{
  servos->setPWM(4, 0, map( angle, 0, 180, servoMIN, servoMAX));
}

//Stepper motor control
void Amc::liftingCmd(int liftCmd)
{ 
  if (liftCmd == 0) {
    steppers->setPWM(1, 0, 4096);
  }
  else {
    steppers->setPWM(1, 0, 2048);
    if (liftCmd == 1) {
      Serial.println("going up");
      steppers->setPWM(0, 4096, 0); 
    }
    else if (liftCmd == -1) {
      Serial.println("going down");
      steppers->setPWM(0, 0, 4096); 
    }
  }
}

void Amc::carouselCmd(int movement)
{
    if(movement==1)
    {
      stepsLeft += 200;
      drop_in = true;
      stepperContinue();
      drop_in_action();
      //dispense();
      //dispense_stack_helper();
      movement = 0;
    }
}

void Amc::stepperContinue()
{
    if (stepsLeft > 0) {
    steppers->setPWM(2, 4096, 0);
    while (stepsLeft > 0) { 
    //if (stepsLeft > 0) {  
      speedControl = stepsLeft % 200;
      //if (stepsLeft != 0) {Serial.println(speedControl);}      
      if (speedControl > 75 && speedControl < 125) {carouselSpeed = 10000;}
      else {carouselSpeed = 1000;}    

      digitalWrite(CarouselPin, HIGH);
      delayMicroseconds(carouselSpeed);
      digitalWrite(CarouselPin, LOW);
      delayMicroseconds(carouselSpeed);    
      stepsLeft -= 1;      
    }
    //Serial.println(stepsLeft);
  }
  else if (stepsLeft < 0) {
    steppers->setPWM(2, 0, 4096);
    while (stepsLeft < 0) {
    //if (stepsLeft < 0) {
      speedControl = stepsLeft % 200;
      //if (stepsLeft != 0) {Serial.println(speedControl);}      
      if (speedControl > -125 && speedControl < -75) {carouselSpeed = 10000;}
      else {carouselSpeed = 1000;}     

      digitalWrite(CarouselPin, HIGH);
      delayMicroseconds(carouselSpeed);
      digitalWrite(CarouselPin, LOW);
      delayMicroseconds(carouselSpeed);    
      stepsLeft += 1;      
    }
    //Serial.println(stepsLeft);
  } 
}

void Amc::activate_paddle() {
  servos->setPWM(4, 0, map( 0, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
  servos->setPWM(4, 0, map( 180, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
}

void Amc::dispense_stack_helper() {
    if (dispense_stack == true && dispensed == false) {
      tube[0] = 0;
      tube[1] = 0;
      tube[2] = 0;        
      dispensed = true;
    } 
}

void Amc::dispense_helper(int i, int pedestal) {
  if (i == 0 && stepsLeft == 0) { //drop pedestal
    activate_paddle();
    slots[0] = 0;
    tube[pedestal - 1] = pedestal; 
  }        
  else {
    if (i == 1 || i == 2) {
      stepsLeft -= 200;
      update_slots(0);
    }
    else {
      stepsLeft += 200;
      update_slots(1);
    }  
  }
     
}

void Amc::dispense() {  
  if (drop_in == false && dispense_stack == false) {
    if (tube[0] == 0) { //empty tube
      for(int i = 0; i < 5; i++) {
        if (slots[i] == 1) { //srearch for white pedestal
          dispense_helper(i, 1);
          break;    
        } 
      }
    }
    else if (tube[0] == 1 && tube[1] == 0) { //white pedetal at bottom
      for(int i = 0; i < 5; i++) {
        if (slots[i] == 2) { //search for green pedestal
          dispense_helper(i, 2);
          break;       
        }
      }
    }  
    else if (built[0] == 0 && tube[0] == 1 && tube[1] == 2 && tube[2] == 0) { 
      //white pedetal at bottom, green in the middle, waitinf for red to build 3 stack
      for(int i = 0; i < 5; i++) {
        if (slots[i] == 3) { //search for red pedestal
          dispense_helper(i, 3);
          break;
        }
      }
    }    
  }  
}

void Amc::drop_in_action() {
  if (drop_in == true){
    if (stepsLeft == 0) {
      //update array after rotation
      update_slots(1);
      //Serial.print("after ccw turn: "); 
            
      //get reading for what's in slot 0
      tcs->getRawData(&r, &g, &b, &c);
      tcs->getRawData(&r, &g, &b, &c);
      if (c > 25000) {slots[0] = 1;} //white
      else if (g > r && g > b) {slots[0] = 2;}     //green
      else {slots[0] = 3;}           //red
         
      drop_in = false;    
    }
  }
}

void Amc::update_slots(int dir) {
  if (dir == 0) //cw
  {
    temp = slots[0];
    slots[0] = slots[1];
    slots[1] = slots[2];
    slots[2] = slots[3];
    slots[3] = slots[4];
    slots[4] = temp;    
  }
  else //ccw
  {
    temp = slots[4];    
    slots[4] = slots[3];      
    slots[3] = slots[2];
    slots[2] = slots[1];
    slots[1] = slots[0];
    slots[0] = temp;
  }
}

void Amc::lowerArm() {

}
void Amc::storeUprightPedestal(const int wristOffset) {
    //open clamp
    deg2 = 180;
    servos->setPWM(1, 0, map( deg2, 0, 180, servoMIN, servoMAX)); //clamp open
    delay(500);
    /*
    //lower arm
    while(!digitalRead(lower_limit)) {
        steppers->setPWM(0, 0, 4096);
    }
    steppers->setPWM(1, 0, 4096);
    */
    //close clamp
    deg2 = 0;
    servos->setPWM(1, 0, map( deg2, 0, 180, servoMIN, servoMAX)); //clamp close
    delay(500);
    //raise arm
    /*
    while(!digitalRead(upper_limit)) {
        steppers->setPWM(0, 4096, 0);
    }
    steppers->setPWM(1, 0, 4096);
    */
    //rotate pedestal for dropping in
    
    //flip arm back
    deg4 = 0;
    servos->setPWM(3, 0, map( deg4, 0, 180,  flipMIN,  flipMAX)); //arm flip
    delay(1000);
    //open clamp
    deg2 = 180;
    servos->setPWM(1, 0, map( deg2, 0, 180, servoMIN, servoMAX)); //clamp open
    delay(1000);
}
