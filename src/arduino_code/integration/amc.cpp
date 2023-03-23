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
    onWhite = true;
    onGreen = false;
    onRed = false; 
    onTwo = false;
    onThree = true;
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
      steppers->setPWM(0, 0, 4096); 
    }
    else if (liftCmd == -1) {
      //Serial.println("going down");
      steppers->setPWM(0, 4096, 0); 
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
      //steppers->setPWM(2, 0, 4096);
    }
}

void Amc::stepperContinue()
{
    if (this->stepsLeft > 0) {
      
        //steppers->setPWM(2, 4096, 0);
        digitalWrite(carouselDir, HIGH);
        while (this->stepsLeft > 0) { 
        if (this->stepsLeft > 0) {  
        this->speedControl = this->stepsLeft % 200;
        //if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}      
        if (this->speedControl > 75 && this->speedControl < 125) {this->carouselSpeed = 10000;}
        else {this->carouselSpeed = 1000;}    
        
        digitalWrite(carouselStep, HIGH);
        delayMicroseconds(5000);
        digitalWrite(carouselStep, LOW);
        delayMicroseconds(5000);    
        this->stepsLeft -= 1;      
    }
    ////Serial.println(this->stepsLeft);
  }
   }
  else if (this->stepsLeft < 0) {
    //steppers->setPWM(2, 0, 4096);
    digitalWrite(carouselDir, LOW);
    while (this->stepsLeft < 0) {
    //if (this->stepsLeft < 0) {
      this->speedControl = this->stepsLeft % 200;
      //if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}      
      if (this->speedControl > -125 && this->speedControl < -75) {this->carouselSpeed = 10000;}
      else {this->carouselSpeed = 1000;}     

      digitalWrite(carouselStep, HIGH);
      delayMicroseconds(5000);
      digitalWrite(carouselStep, LOW);
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
    else if (this->tube[0] == 1 && (this->tube[1] == 0 || this->tube[2] == 0)) { //white pedetal at bottom
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

      if (this->onWhite) {
        if(this->slots[0] == 1){
          dispense_helper(0,1);
          this->onWhite = false;
          this->onGreen = true;
        }
      }
      else if (this->onGreen) {
        if(this->slots[0] == 2){
          dispense_helper(0,2);
          this->onGreen = false;
          this->onRed = false;
        }
      }
      else if (this->onRed) {
        this->onRed = false;
        if(this->slots[0] == 3){
          dispense_helper(0,3);
        }
       
      }
    }
  //Now need to check if the 4th slot is empty

  //first will check if there is a white in tube, if there is will find green and dispense it
  //then we will check if we have stack in tube if we do then we will not dispense anything we will only make sure slot4 is empy
  
  if(this->slots[4] != 0 && !this->slotsFull) {
    
    //first find a green if there is a white in tube and there isnt a green already
    if(this->tube[0] == 1 && this->tube[1] == 0){
      //dispense a green 
      for(int i =0; i<5; i++){
        if(this->slots[i] == 2){
          dispense_helper(i,2);
          break;
        }
      }
    }
    
    //then we will check if the carousel is full we wont do anything just get out of function
    //beasue it is not empy we will check
    int emptySlotIndex = findEmptySlot();
    
    if(emptySlotIndex == -1){
      //this means not empty slots, carousel full
      this->slotsFull = true;
      return;
    }
    else{
      //so if it is not full then we will go to the empty slot
      initSlotFour(emptySlotIndex);
    }
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
  
  if(this->slots[4] == 0){
    this->loadSlotEmpty = true;
  }
  else {
    this->loadSlotEmpty = false;
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

int Amc::findEmptySlot(){
  if (this->slots[4] == 0){
    return 4;
  }
  else{
    for (int i = 0; i < 4; i++){
      if (this->slots[i] == 0){
        return i;
      }
    }
  }
  //if nothing empty we return -1
  return -1;
  
}

void Amc::initSlotFour(int index){
  if(index == 4){
    return;
  }

  if (index == 0 || index == 1) {
      for (int j = 0; j < index; j++) {
        this->stepsLeft -= 200;
        stepperContinue();
        update_slots(0);
      }
    }
    
    if (index == 2 || index == 3 ) {
      for (int k = 2; k < index+1; k++) {
        this->stepsLeft += 200;
        stepperContinue();
        update_slots(1);
      }  
    }
}
