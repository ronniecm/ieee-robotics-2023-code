#include <util/atomic.h>
#include <PID_v1.h>
#include <TimerOne.h>

#define ENA_1 0
#define ENB_1 5

#define ENA_2 6
#define ENB_2 11

#define IN1_1 1
#define IN2_1 2
#define IN3_1 3
#define IN4_1 4

#define IN1_2 7
#define IN2_2 8
#define IN3_2 9
#define IN4_2 10

#define ENCA_1 27 
#define ENCB_1 28

#define ENCA_2 31
#define ENCB_2 32

#define ENCA_3 37
#define ENCB_3 38

#define ENCA_4 33
#define ENCB_4 34

#define ENCPOWER 39

/*
double kP = 5;
double kI = 10;
double kD = 0;
double setpoint, input, output, toMotor;
PID pidController(&input, &output, &setpoint, kP, kI, kD, DIRECT);

float demand= 0;
*/

int pos_i[4];
unsigned long previousTime[4];
int prevPos[4];
float rpm[4];

/*
volatile float velocity_i = 0;
volatile long prevT_i;

unsigned long currentMillis;
unsigned long previousMillis;
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA_1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(ENB_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  
  pinMode(ENCA_1, INPUT);
  pinMode(ENCB_1, INPUT);
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_2, INPUT);

  pinMode(ENA_2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(ENB_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  
  pinMode(ENCA_3, INPUT);
  pinMode(ENCB_3, INPUT);
  pinMode(ENCA_4, INPUT);
  pinMode(ENCB_4, INPUT);

  pinMode(ENCPOWER, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCB_1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_4), readEncoder4, RISING);


  /*
  pidController.SetMode(AUTOMATIC);
  pidController.SetOutputLimits(-100, 100);
  pidController.SetSampleTime(10);
  */
  Serial.begin(9600);
  digitalWrite(ENCPOWER, HIGH);
  //Timer1.initialize(1000);
  //Timer1.attachInterrupt(calcRPM);
}


void loop() {
  forward(0.5);
  int pos[4];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(int i = 0; i < 4; i++) {
        pos[i] = pos_i[i];
      }  
  }
  for(int i = 0; i < 4; i++) {
    Serial.print(pos[i]);
    Serial.print(" ");  
  }
  Serial.println();
}

void calcRPM() {
  for(int i = 0; i < 4; i++) {
    int deltaPos = pos_i[i] - prevPos[i];
    double revs = deltaPos / 3000.0;
    double rpm = revs * 60000.0;
    Serial.print(rpm);
    Serial.print(" ");
    prevPos[i] = pos_i[i];
  }
  Serial.println();  
}

void rotateL(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed);
}

void rotateR(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed);
}

void reverse(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed);  
}

void forward(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed);  
}

void strafeR(float norm) {
   int speed = (norm) * 255;
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);
  analogWrite(ENB_2, speed); 
}

void strafeL(float norm) {
  int speed = (norm) * 255;
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);
  analogWrite(ENA_1, speed);

  digitalWrite(IN3_1, HIGH);
  digitalWrite(IN4_1, LOW);
  analogWrite(ENB_1, speed);

  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_2, speed);

  digitalWrite(IN3_2, HIGH);
  digitalWrite(IN4_2, LOW);
  analogWrite(ENB_2, speed); 
}

void stopBot() {
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, LOW);
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, LOW);
}  

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[0] += increment;
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[1] += increment;
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_3);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[2] += increment;
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_4);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[3] += increment;
}
