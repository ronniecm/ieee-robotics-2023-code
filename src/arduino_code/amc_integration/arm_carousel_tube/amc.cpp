#include "amc.h"

Amc::Amc() {    
    servos = new Adafruit_PWMServoDriver(0x40);
    steppers = new Adafruit_PWMServoDriver(0x41);
    tcs = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

    servos->begin();
    servos->setPWMFreq(50);
  
    steppers->begin();
    steppers->setPWMFreq(1000);
    
    if (tcs->begin()) {
        Serial.println("Found sensor");
    }

    deg1 = 0; //pedestal rotate
    deg2 = 0; //clamp close
    deg3 = 180; //clamp pan
    deg4 = 180; //arm flip
    deg5 = 180; //paddle
    deg6 = 100; // door
    lift = 0; //lifting
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
      Serial.print("tube: ");
      for(int i = 0; i < 3; i++){Serial.print(tube[i]);}
      Serial.println();          
      dispensed = true;
    } 
}

void Amc::dispense_helper(int i, int pedestal) {
  if (i == 0 && stepsLeft == 0) { //drop pedestal
    activate_paddle();
    slots[0] = 0;
    tube[pedestal - 1] = pedestal;
    Serial.print("tube: ");
    for(int i = 0; i < 3; i++){Serial.print(tube[i]);}
    Serial.println();        
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
      Serial.print("dispense update: ");
      for(int i = 0; i < 5; i++){Serial.print(slots[i]);}
      Serial.println();      
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
      Serial.print("after ccw turn: ");
      for(int i = 0; i < 5; i++){Serial.print(slots[i]);}
      Serial.println();   
            
      //get reading for what's in slot 0
      tcs->getRawData(&r, &g, &b, &c);
      tcs->getRawData(&r, &g, &b, &c);
      if (c > 25000) {slots[0] = 1;} //white
      else if (g > r && g > b) {slots[0] = 2;}     //green
      else {slots[0] = 3;}           //red
      Serial.print("get reading for slot 0: ");
      for(int i = 0; i < 5; i++){Serial.print(slots[i]);}
      Serial.println();          
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