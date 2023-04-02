//#include <stdint.h>
#include "amc.h"

#define flipMIN 115
#define flipMAX 520

#define wristMIN 120
#define wristMAX 530

#define servoMIN 103
#define servoMAX 512

#define upper_limit 39  //upper limit switch
#define lower_limit 38  //lower limit switch

#define carouselDir 23
#define carouselStep 22

#define elevatorDir 20
#define elevatorStep 21

#define WHITE 1
#define GREEN 2
#define RED 3
#define EMPTY 0


Amc::Amc()
{
  servos = new Adafruit_PWMServoDriver(0x41);
  servos->begin();
  servos->setPWMFreq(50);

  tcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
  //Wire2.begin();
  tcs->begin(0x29, &Wire2);
  
  
  //Serial.print("Color Sensor");
  //foodChipTcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

  
  // color sensor
  
  //foodChipTcs->begin(0x29, &Wire1);


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


void Amc::liftingCmd(int liftCmd)
{
  
  if (liftCmd == 1) {
    Serial.print("In Lifting");
    digitalWrite(elevatorDir, LOW);
    while(digitalRead(upper_limit) == HIGH) {
      digitalWrite(elevatorStep, HIGH);
      delayMicroseconds(1000);
      digitalWrite(elevatorStep, LOW);
      delayMicroseconds(1000); 
    }
  }
  else if (liftCmd == -1) {
    digitalWrite(elevatorDir, HIGH);
    while(digitalRead(lower_limit) == HIGH) {
      digitalWrite(elevatorStep, HIGH);
      delayMicroseconds(1000);
      digitalWrite(elevatorStep, LOW);
      delayMicroseconds(1000);
    }
  }  
  else{
    return;
  }

}


void Amc::carouselCmd(int stackType)
{

    //Serial.println("in carousel");
  
    if (stackType == 2 || stackType ==3) {
      
      if(stackType ==2){
        this->onTwo = true;
      }

      if(stackType ==3){
        this->onThree = true;
      }
      
      fillStack(stackType);
    }
     
    else if (stackType == 1)
    {
      
      this->stepsLeft -= 200;
      stepperContinue();
      this->drop_in = true;
      drop_in_action();
       
    }
    else
    {
      //Serial.println("HERE");
      return;
    }

}


void Amc::fillStack(int stackType)
{
  //dispense function gets a single desired pedestal in tube,
  //Therefore dispense needs to be called 2 times for 2 stack and 3 for 3 stack

  Serial.println("Fillstack");
  for (int i = 0; i < stackType; i++)
  {
    dispense();
  }

  this->dispense_stack = true;
  //This will clear the tube making sure we are ready to dispense the next stack
  dispense_stack_helper();

  // will check to see if there is a green pedestal"2" in the second indec of the tube then mark it as built
  if ((stackType == 2) && (this->built[2] == 0) && (this->tube[1] == GREEN))
  {
    this->built[2] = 1;
    return;
  }

  if ((stackType == 3) && (this->built[0] == 0) && (this->tube[2] == RED))
  {
    this->built[0] = 1;
    return;
  }
}


void Amc::stepperContinue()
{

    Serial.print("STEPPER CONTINUE");
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
      // if (this->stepsLeft != 0) {//Serial.println(this->speedControl);}
      if (this->speedControl > -125 && this->speedControl < -75)
      {
        this->carouselSpeed = 10000;
      }
      else
      {
        this->carouselSpeed = 1000;
      }

      digitalWrite(carouselStep, HIGH);
      delayMicroseconds(5000);
      digitalWrite(carouselStep, LOW);
      delayMicroseconds(5000);    
      this->stepsLeft += 1;      
    }
    ////Serial.println(this->stepsLeft);
  }
}



void Amc::drop_in_action()
{

  if (this->drop_in == true)
  {

    if (this->stepsLeft == 0)
    {
      // update array after rotation

      update_slots(1);
      ////Serial.print("after ccw turn: ");

      // get reading for what's in slot 0
      tcs->getRawData(&r, &g, &b, &c);
      tcs->getRawData(&r, &g, &b, &c);

      Serial.print("Red: ");
      Serial.print(r);
      Serial.print(" Green: ");
      Serial.print(g);
      Serial.print(" Blue: ");
      Serial.print(b);
      Serial.print(" Clear: ");
      Serial.println(c);
      
      
      if (g > r && g > b && c < 3200 && c > 2000 )
      {
        this->slots[0] = GREEN;
      } // green
      else if (r > g && r > b && c < 2000 && c > 1000)
      {
        this->slots[0] = RED;
      } // red
      else
      {
        this->slots[0] = WHITE;
      } // whtie


      
      this->drop_in = false;

      if (this->onWhite && this->built[1] == 0) {
        if(this->slots[0] == WHITE){
          dispense_helper(0,WHITE);
 
          this->onWhite = false;
          this->onGreen = true;
        }
      }
      else if (this->onGreen && this->built[1] == 0) {
        
        if(this->slots[0] == GREEN){
          dispense_helper(0,GREEN);
          

          this->onGreen = false;
          if(this->onTwo){
            built[1] = 1;
            this->onTwo = false;
            this->onWhite = true;
            this->dispense_stack = true;
            dispense_stack_helper();
          }
          else{
            this->onRed = true;
          }
        }
      }
      else if (this->onRed) {
        
        if(this->slots[0] == RED){
          dispense_helper(0,RED);
          this->onRed = false;
          this->onWhite = true;
        }
      }
    }
  // Now need to check if the 4th slot is empty
  //first will check if there is a white in tube, if there is will find green and dispense it
  //then we will check if we have stack in tube if we do then we will not dispense anything we will only make sure slot4 is empy
  
  int emptySlotIndex = findEmptySlot();
 
  //Will return index of emptty slot, if carousel ful will return -1
  if(this->slots[4] != EMPTY && emptySlotIndex != -1) {
    
    //first find a green if there is a white in tube and there isnt a green already
    if(this->onGreen){
      //dispense a green 
      dispense_helper(findPedestalSlot(GREEN),GREEN);
      this->onGreen = false;
      
      if(this->onTwo){
        this->dispense_stack = true;
        dispense_stack_helper();
        built[1] = 1;
        this->onWhite = true;
        this->onRed = false;
      }
      else{
        this->onRed = true;
      }
    }
    
    //then we will check if the carousel is full we wont do anything just get out of function
    //beasue it is not empy we will check
    initSlotFour(findEmptySlot());
  }
 }
}



void Amc::dispense()
{
  Serial.print("IN DISPENSE");
  if (this->drop_in == false && this->dispense_stack == false)
  {
    if (this->onWhite)
    { 
      dispense_helper(findPedestalSlot(WHITE), WHITE);
      this->onGreen = true;
      this->onWhite = false;
      return;
    }
    else if (this->onGreen)
    { // white pedetal at bottom
      dispense_helper(findPedestalSlot(GREEN), GREEN);
      this->onGreen = false;
      
      
      if(this->onTwo){
        this->onTwo = false;
        this->onWhite = true;
      }
      else{
        this->onWhite = false;
        this->onRed =true;
        
        
      }      
      return;
    }
    
    else if (built[0] == 0 && this->onRed)
    {
      if(this->built[2] == 0){
        this->onWhite = true;
      }
      
      // white pedetal at bottom, green in the middle, waitinf for red to build 3 stack
      dispense_helper(findPedestalSlot(RED), RED);
      return;
    }
  }
}

void Amc::dispense_stack_helper()
{
  if (this->dispense_stack)
  {
    this->tube[0] = 0;
    this->tube[1] = 0;
    this->tube[2] = 0;
    // ADD REMOVING THE BUILT INDEX OF THE STACK WE WANT BUILT BECAUASE WE HAVE THAT IN ROS MSG
    this->dispense_stack = false;
  }
}

void Amc::update_slots(int dir)
{

  int temp = 0;

  if (dir == 0) // cw
  {
    temp = this->slots[0];
    for (int i = 0; i < 4; i++)
    {
      this->slots[i] = this->slots[i + 1];
    }
    this->slots[4] = temp;
  }

  else // ccw
  {
    temp = this->slots[4];
    for (int i = 4; i > 0; i--)
    {
      this->slots[i] = this->slots[i - 1];
    }

    this->slots[0] = temp;
  }

}



void Amc::dispense_helper(int index, int pedestal)
{
  if (index == 0 && this->stepsLeft == 0)
  { // drop pedestal
    activate_paddle();
    this->slots[0] = EMPTY;
    this->tube[pedestal - 1] = pedestal;
  }
  else
  {
    if (index == 1 || index == 2)
    {
      //Number of clockwise rotations depends on the n
      for (int i = 0; i < index; i++)
      {
        
        this->stepsLeft += 200;
        stepperContinue();
        update_slots(0);
      }
      activate_paddle();
      this->slots[0] = EMPTY;
      this->tube[pedestal - 1] = pedestal;
    }

    if (index == 3 || index == 4)
    {
      //Subtracting from index from 5 becuase that gives use number slots away
      //from slot 0. Number of slots away is number of slots we want to rotates carousel
      for (int i = 0; i < 5-index; i++)
      {
        //Counter ClockWise Turn
        this->stepsLeft -= 200;
        stepperContinue();
        update_slots(1);
      }
      activate_paddle();
      this->slots[0] = EMPTY;
      this->tube[pedestal - 1] = pedestal;
    }
  }
}

int Amc::findEmptySlot(){
  if (this->slots[4] == EMPTY){
    return EMPTY;
  }
  else{
    //We are going to check the most adjacent slots first becuase 
    //These require the least amount of turns
    //Plus doing this way we only need to iterate through for loop twice instead of 4
    for (int index = 0; index < 2; index++){
      if (this->slots[index] == EMPTY){
        return index;
      }
      if(this->slots[3-index] == EMPTY){
        return 3-index;
      }
    }
  }
  // if nothing empty we return -1
  return -1;
}

int Amc::findPedestalSlot(int pedestal){
  //We are going to check the most adjacent slots first becuase 
  //These require the least amount of turns
  //Plus doing this way we only need to iterate through for loop twice instead of 4
  if (this->slots[0] == pedestal){
    return 0;
  }
  else{
    for (int index = 1; index <= 2; index++){
      if (this->slots[index] == pedestal){
        return index;
      }
      if(this->slots[5-index] == pedestal){
        return 5-index;
      }
    }
  }
  // if pedestal not found we return -1
  return -1;

}


void Amc::initSlotFour(int index){
  if(index == 4){
    return;
  }

  if (index == 0 || index == 1)
  {
    //0 index is 1 turn away 1 index is 2 turns away
    //so we will do 1 turn for 0 index and 2 turns for 1 index
    for (int i = 0; i <= index; i++)
    {
      this->stepsLeft += 200;
      stepperContinue();
      update_slots(0);
    }
  }

  //if index is 2 or 3 we will do the same thing as above but in the other direction
  if (index == 2 || index == 3)
  {
    // (4-index) check how many slots we are away from slot 4. so that is why it is in the for
    //loop condistion. Basically will tell us how many turns to make
    for (int i = 0; i < 4-index; i++)
    {
      this->stepsLeft -= 200;
      stepperContinue();
      update_slots(1);
    }
  }
}

void Amc::activate_paddle()
{
  // Serial.println("Dropping");
  servos->setPWM(4, 0, map(0, 0, 180, servoMIN, servoMAX)); // paddle
  delay(2000);
  servos->setPWM(4, 0, map(135, 0, 180, servoMIN, servoMAX)); // paddle
  
}

void Amc::getColorData()
{
  tcs->getRawData(&r, &g, &b, &c);
  
  Serial.print("Red: ");
  Serial.print(r);
  Serial.print(" Green: ");
  Serial.print(g);
  Serial.print(" Blue: ");
  Serial.print(b);
  Serial.print(" Clear: ");
  Serial.println(c);
}
