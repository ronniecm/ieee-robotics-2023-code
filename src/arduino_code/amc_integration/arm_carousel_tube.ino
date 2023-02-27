#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Encoder.h>
//#include <nRF24L01.h>
//#include <RF24.h>
#include "Adafruit_TCS34725.h"

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver steppers = Adafruit_PWMServoDriver(0x41);

#define servoMIN 103
#define servoMAX 512

#define flipMIN 125
#define flipMAX 530

#define liftingSpeed 200
#define stepsPerRevolution 200

#define CarouselPin 33
#define red_TH 5000
#define green_TH 25000

#define upper_limit 39
#define lower_limit 38

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

char input;
int deg1 = 0; //pedestal rotate
int deg2 = 0; //clamp close
int deg3 = 180; //clamp pan
int deg4 = 180; //arm flip
int deg5 = 180; //paddle
int deg6 = 100; // door
int lift = 0; //lifting
int stepsLeft = 0;
int carouselSpeed = 1000;
int speedControl = 0;
int slots[] = {0, 0, 0, 0, 0, }; //slot assignment: start from tube slot & ccw: 0, 1, 2, 3, 4
int tube[] = {0, 0, 0};
int built[] = {0, 0, 0}; //[0] for 3 stack, [1],[2] for 2 stacks
int temp;
bool drop_in = false;
bool dispense_stack= false;
bool dispensed = true;
bool dispense_stack_start = false;
bool dispense_stack_finish = false;
int drive_state = 1; //0: stop; 1: forward; 2:backward

uint16_t r, g, b, c, colorTemp, lux;

//create an RF24 object
//RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
//const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);

  servos.begin();
  servos.setPWMFreq(50);
  
  steppers.begin();
  steppers.setPWMFreq(1000);

  //radio.begin();
  
  //set the address
  //radio.openReadingPipe(0, address);
  
  //Set module as receiver
  //radio.startListening();
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  }  

  //pinMode(34, OUTPUT);
  //pinMode(35, INPUT);
  pinMode(34, INPUT);
  pinMode(13, OUTPUT);
}

void loop() {
    //Read the data if available in buffer
  /*
  if (radio.available())
  {
    //char text[32] = {0};
    char text;
    //radio.read(&text, sizeof(text));
    radio.read(&text, 1);
    Serial.println(text);
    //if (text == 'o') {
    //  if (deg1 == 0) {deg1 = 90;}
    //  else {deg1 = 0;}
    //}    
    if (text == 'w') {
      if (drive_state == 1) {drive_state = 0;}
      else {drive_state = 1;}
    }
    else if (text == 's') {
      if (drive_state == 2) {drive_state = 0;}
      else {drive_state = 2;}
    }
  }
  */
  
  if (Serial.available()) {
    input = Serial.read(); 
    if (input == 'o') {
      if (deg1 == 0) {deg1 = 90;}
      else {deg1 = 0;}
    }
    else if (input == 'u') {
      if (deg2 == 0) {deg2 = 180;}
      else {deg2 = 0;}  
      //Serial.println(deg2);  
    }
    else if (input == 'j') {
      deg3 += 30;
      if (deg3 > 180) {
        deg3 = 180;
      }
    }
    else if (input == 'l') {
      deg3 -= 30;
      if (deg3 < 0) {
        deg3 = 0;
      }
    }    
    else if (input == 'i') {
      deg4 = 0;
    }
    else if (input == 'k') {
      deg4 = 180;
    }     
    else if (input == 'w') {
      if (lift == 0) {lift = 1;}
      else {lift = 0;}  
    }   
    else if (input == 's') {
      if (lift == 0) {lift = -1;}
      else {lift = 0;}    
    } 
    else if (input == 'e') {stepsLeft += 200;}
    else if (input == 'q') {stepsLeft -= 200;}
    else if (input == 'c') 
    {
      tcs.getRawData(&r, &g, &b, &c);
      //Serial.println(c);
      Serial.print(r);
      Serial.print(" | ");
      Serial.print(g);
      Serial.print(" | ");
      Serial.print(b);
      Serial.print(" | ");
      Serial.print(c);
      Serial.println();
      if (c > 25000) {Serial.println("white");} //white
      else if (g > r && g > b) {Serial.println("green");}     //green
      else {Serial.println("red");}           //red      
    }
    else if (input == '1') 
    {
      //print origin
      Serial.print("origin: ");
      for(int i = 0; i < 5; i++){Serial.print(slots[i]);}
      Serial.println(); 
      
      //make ccw turn
      stepsLeft += 200;  

      //update slot after turn & get reading for pedestal in slot 0
      drop_in = true;      
    }
    else if (input == '2') {
      dispense_stack = true;  
      dispensed = false;
      deg6 = 0;      
    }
    else if (input == '3') {
      dispense_stack = false;  
      deg6 = 100;
    }    
    else if (input == 'p') {
      if (deg5 == 0) {deg5 = 180;}
      else {deg5 = 0;}       
    }
    else if (input == 'd') {
      if (deg6 == 0) {deg6 = 100;}
      else {deg6 = 0;}       
    }    
    //Serial.println(stepsLeft);
  } 

  //            ID        degree
  servos.setPWM(0, 0, map( deg1, 0, 180, servoMIN, servoMAX)); //pedestal rotate
  servos.setPWM(1, 0, map( deg2, 0, 180, servoMIN, servoMAX)); //clamp close
  servos.setPWM(2, 0, map( deg3, 0, 180, servoMIN, servoMAX)); //clamp pan
  servos.setPWM(3, 0, map( deg4, 0, 180,  flipMIN,  flipMAX)); //arm flip
  servos.setPWM(4, 0, map( deg5, 0, 180, servoMIN,  servoMAX)); //paddle
  servos.setPWM(5, 0, map( deg6, 0, 180, servoMIN,  servoMAX)); //door

  if (digitalRead(lower_limit) == HIGH) {digitalWrite(13,LOW);}
  else {digitalWrite(13,HIGH);};
  if (lift == 1 && digitalRead(upper_limit) == LOW){lift = 0;} 
  if (lift == -1 && digitalRead(lower_limit) == LOW){lift = 0;}

  if (lift == 0) {
    steppers.setPWM(1, 0, 4096);
  }
  else {
    steppers.setPWM(1, 0, 2048);
    if (lift == 1) {
      steppers.setPWM(0, 4096, 0); 
    }
    else if (lift == -1) {
      steppers.setPWM(0, 0, 4096); 
    }
  }

  speedControl = stepsLeft % 200;
  if (stepsLeft != 0) {Serial.println(speedControl);}
  //if ((speedControl > 75 && speedControl < 125) || (speedControl > -125 && speedControl < -75)) {carouselSpeed = 5000;}
  //else {carouselSpeed = 1000;}
  
  if (stepsLeft > 0) {
    steppers.setPWM(2, 4096, 0);
    while (stepsLeft > 0) { 
    //if (stepsLeft > 0) {  
      speedControl = stepsLeft % 200;
      if (stepsLeft != 0) {Serial.println(speedControl);}      
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
    steppers.setPWM(2, 0, 4096);
    while (stepsLeft < 0) {
    //if (stepsLeft < 0) {
      speedControl = stepsLeft % 200;
      if (stepsLeft != 0) {Serial.println(speedControl);}      
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

  drop_in_action();
  dispense();
  dispense_stack_helper();
}

//void loop ends

void update_slots(int dir) {
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

void drop_in_action() {
  if (drop_in == true){
    if (stepsLeft == 0) {
      //update array after rotation
      update_slots(1);
      Serial.print("after ccw turn: ");
      for(int i = 0; i < 5; i++){Serial.print(slots[i]);}
      Serial.println();   
            
      //get reading for what's in slot 0
      tcs.getRawData(&r, &g, &b, &c);
      tcs.getRawData(&r, &g, &b, &c);
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

void dispense() {  
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

void dispense_helper(int i, int pedestal) {
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

void activate_paddle() {
  servos.setPWM(4, 0, map( 0, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
  servos.setPWM(4, 0, map( 180, 0, 180, servoMIN,  servoMAX)); //paddle
  delay(1000);
}

void dispense_stack_helper() {
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
