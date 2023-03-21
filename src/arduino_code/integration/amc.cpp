#include "amc.h"

#define flipMIN 120
#define flipMAX 530

#define wristMIN 110
#define wristMAX 520

#define servoMIN 103
#define servoMAX 512
#define upper_limit 39  //upper limit switch
#define lower_limit 38  //lower limit switch
#define CarouselPin 13

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

void Amc::carouselCmd(int stackType)
{   

    if (stackType == 2 || stackType ==3) {
      fillStack(stackType);
      
    }
     
    else if (stackType == 1)
    {
      this->stepsLeft += 200;
      stepperContinue();
      this->drop_in = true;
      drop_in_action();
      
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
        if (this->stepsLeft > 0) {  
        this->speedControl = this->stepsLeft % 200;
        //if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}      
        if (this->speedControl > 75 && this->speedControl < 125) {this->carouselSpeed = 10000;}
        else {this->carouselSpeed = 1000;}    
        
        digitalWrite(CarouselPin, HIGH);
        delayMicroseconds(5000);
        digitalWrite(CarouselPin, LOW);
        delayMicroseconds(5000);    
        this->stepsLeft -= 1;      
    }
    ////Serial.println(this->stepsLeft);
  }
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
      delayMicroseconds(5000);
      digitalWrite(CarouselPin, LOW);
      delayMicroseconds(5000);    
      this->stepsLeft += 1;      
    }
    ////Serial.println(this->stepsLeft);
  } 

}

void Amc::activate_paddle() {
  //Serial.println("Dropping");
  servos->setPWM(4, 0, map( 0, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
  servos->setPWM(4, 0, map( 135, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
}

void Amc::dispense_stack_helper() {
    if (this->dispense_stack) {
      this->tube[0] = 0;
      this->tube[1] = 0;
      this->tube[2] = 0;   
      //ADD REMOVING THE BUILT INDEX OF THE STACK WE WANT BUILT BECAUASE WE HAVE THAT IN ROS MSG     
      this->dispense_stack = false;
    } 
}

void Amc::dispense_helper(int i, int pedestal) {
  //Serial.print("Index: ");
  //Serial.println(i);
  if (i == 0 && this->stepsLeft == 0) { //drop pedestal
    activate_paddle();
    this->slots[0] = 0;
    this->tube[pedestal - 1] = pedestal; 
    
  }        
  else {
    if (i == 1 || i == 2) {
      for (int j = 0; j < i; j++) {
        //Serial.println("Clockwise");
        this->stepsLeft -= 200;
        stepperContinue();
        update_slots(0);
      }
      activate_paddle();
      this->slots[0] = 0;
      this->tube[pedestal - 1] = pedestal; 
    }
    
    if (i == 3 || i == 4 ) {
      for (int k = 2; k < i+1; k++) {
        //Serial.println("CounterClockwise");
        this->stepsLeft += 200;
        stepperContinue();
        update_slots(1);
      }  
      activate_paddle();
      this->slots[0] = 0;
      this->tube[pedestal - 1] = pedestal; 
    }
  }   
}

void Amc::dispense() {  
  
  //Serial.println("In Dispense");
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
      
      if (g > r && g > b && abs(r - g) > 500) {this->slots[0] = 2;} //green
      else if (r > g && r > b && abs(r - g) > 500) {this->slots[0] = 3;}     //red
      else {this->slots[0] = 1;}           //whtie
         
      this->drop_in = false;    
    }
  }
}

void Amc::update_slots(int dir) {
  
  int temp = 0;
  
  if (dir == 0) //cw
  {
    temp = this->slots[0];
    for(int i = 0; i < 4; i++){
      this->slots[i] = this->slots[i+1];
    }
    this->slots[4] = temp;
  }
  
  else //ccw
  { 
    temp = this->slots[4];
    for(int i = 4; i > 0; i--){
      this->slots[i] = this->slots[i-1];
    }
   
    this->slots[0] = temp;

  }
}


int Amc::getStepsLeft() {
  return this->stepsLeft;  
}

int* Amc::getSlots() {
  
  return this->slots;
}

void Amc::fillStack(int stackType) {
  
  
  for(int i = 0; i<stackType; i++){
    dispense();
  }
  
  this->dispense_stack = true;
  dispense_stack_helper();
  
  
  //will check to see if there is a green pedestal"2" in the second indec of the tube then mark it as built
  if((stackType==2) && (this->built[1] == 0) && (this->tube[1] == 2)){
    this->built[1] = 1;
    return;
  }
  
  if((stackType==2) && (this->built[2] == 0) && (this->tube[1] == 2)){
    this->built[2] = 1;
    return;
  }

  if((stackType==3) && (this->built[0] == 0) && (this->tube[2] == 3)){
    this->built[0] = 1;
    return;
  }
  
}
