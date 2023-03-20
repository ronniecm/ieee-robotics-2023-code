#include "amc.h"

#define flipMIN 120
#define flipMAX 530

#define wristMIN 110
#define wristMAX 520

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
    //color sensor
    tcs->begin();
  
    //color sensor stuff:    
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
  servos->setPWM(2, 0, map( angle, 0, 270, wristMIN, wristMAX));
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
      //Serial.println("going up");
      steppers->setPWM(0, 4096, 0); 
    }
    else if (liftCmd == -1) {
      //Serial.println("going down");
      steppers->setPWM(0, 0, 4096); 
    }
  }
}

void Amc::carouselCmd(int movement)
{
    if(movement==1)
    {
      this->stepsLeft += 200;
      this->drop_in = true;
      stepperContinue();
      drop_in_action();
      //dispense();
      //this->dispense_stack_helper();
      movement = 0;
    }

    else
    {
      steppers->setPWM(2, 0, 4096);
    }
}

void Amc::stepperContinue()
{
    if (this->stepsLeft > 0) {
      steppers->setPWM(2, 4096, 0);
      while (this->stepsLeft > 0) { 
      //if (this->stepsLeft > 0) {  
        //this->speedControl = this->stepsLeft % 200;
        //if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}      
        //if (this->speedControl > 75 && this->speedControl < 125) {this->carouselSpeed = 10000;}
        //else {this->carouselSpeed = 1000;}    
        
        digitalWrite(CarouselPin, HIGH);
        delayMicroseconds(this->carouselSpeed);
        digitalWrite(CarouselPin, LOW);
        delayMicroseconds(this->carouselSpeed);    
        this->stepsLeft -= 1;      
    }
    ////Serial.println(this->stepsLeft);
  }
  else if (this->stepsLeft < 0) {
    steppers->setPWM(2, 0, 4096);
    while (this->stepsLeft < 0) {
    //if (this->stepsLeft < 0) {
      this->speedControl = this->stepsLeft % 200;
      //if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}      
      if (this->speedControl > -125 && this->speedControl < -75) {this->carouselSpeed = 10000;}
      else {this->carouselSpeed = 1000;}     

      digitalWrite(CarouselPin, HIGH);
      delayMicroseconds(this->carouselSpeed);
      digitalWrite(CarouselPin, LOW);
      delayMicroseconds(this->carouselSpeed);    
      this->stepsLeft += 1;      
    }
    ////Serial.println(this->stepsLeft);
  } 
}

void Amc::activate_paddle() {
  servos->setPWM(4, 0, map( 0, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
  servos->setPWM(4, 0, map( 180, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
}

void Amc::dispense_stack_helper() {
    if (this->dispense_stack == true && this->dispensed == false) {
      this->tube[0] = 0;
      this->tube[1] = 0;
      this->tube[2] = 0;        
      this->dispensed = true;
    } 
}

void Amc::dispense_helper(int i, int pedestal) {
  if (i == 0 && this->stepsLeft == 0) { //drop pedestal
    activate_paddle();
    this->slots[0] = 0;
    this->tube[pedestal - 1] = pedestal; 
  }        
  else {
    if (i == 1 || i == 2) {
      this->stepsLeft -= 200;
      stepperContinue();
      update_slots(0);
    }
    else {
      this->stepsLeft += 200;
      stepperContinue();
      update_slots(1);
    }  
  }
     
}

void Amc::dispense() {  
  if (this->drop_in == false && this->dispense_stack == false) {
    if (this->tube[0] == 0) { //empty this->tube
      for(int i = 0; i < 5; i++) {
        if (this->slots[i] == 1) { //srearch for white pedestal
          dispense_helper(i, 1);
          break;    
        } 
      }
    }
    else if (this->tube[0] == 1 && this->tube[1] == 0) { //white pedetal at bottom
      for(int i = 0; i < 5; i++) {
        if (this->slots[i] == 2) { //search for green pedestal
          dispense_helper(i, 2);
          break;       
        }
      }
    }  
    else if (built[0] == 0 && this->tube[0] == 1 && this->tube[1] == 2 && this->tube[2] == 0) { 
      //white pedetal at bottom, green in the middle, waitinf for red to build 3 stack
      for(int i = 0; i < 5; i++) {
        if (this->slots[i] == 3) { //search for red pedestal
          dispense_helper(i, 3);
          break;
        }
      }
    }    
  }  
}

void Amc::drop_in_action() {
  if (this->drop_in == true){
    if (this->stepsLeft == 0) {
      //update array after rotation
      update_slots(1);
      ////Serial.print("after ccw turn: "); 
            
      //get reading for what's in slot 0
      tcs->getRawData(&r, &g, &b, &c);
      tcs->getRawData(&r, &g, &b, &c);
      
      if (c > 25000) {this->slots[0] = 1;} //white
      else if (g > r && g > b) {this->slots[0] = 2;}     //green
      else {this->slots[0] = 3;}           //red
         
      this->drop_in = false;    
    }
  }
}

void Amc::update_slots(int dir) {
  if (dir == 0) //cw
  {
    this->temp = this->slots[0];
    this->slots[0] = this->slots[1];
    this->slots[1] = this->slots[2];
    this->slots[2] = this->slots[3];
    this->slots[3] = this->slots[4];
    this->slots[4] = temp;    
  }
  else //ccw
  {
    this->temp = this->slots[4];    
    this->slots[4] = this->slots[3];      
    this->slots[3] = this->slots[2];
    this->slots[2] = this->slots[1];
    this->slots[1] = this->slots[0];
    this->slots[0] = this->temp;
  }
}


int Amc::getStepsLeft() {
  return this->stepsLeft;  
}

int* Amc::getSlots() {
  
  return this->slots;
}
