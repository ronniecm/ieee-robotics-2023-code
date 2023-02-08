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

#define ENCA_2 37
#define ENCB_2 38

#define ENCA_3 31
#define ENCB_3 32

#define ENCA_4 33
#define ENCB_4 34

#define ENCPOWER 39

double kP[4] = {10,8  ,10,9};
double kI = 0.0;
double kD[4] = {0,0,0,0};
double setpoint[4];
double in[4];
double out[4];

PID pidController1(&in[0], &out[0], &setpoint[0], kP[0], kI, kD[0], DIRECT);
PID pidController2(&in[1], &out[1], &setpoint[1], kP[1], kI, kD[1], DIRECT);
PID pidController3(&in[2], &out[2], &setpoint[2], kP[2], kI, kD[2], DIRECT);
PID pidController4(&in[3], &out[3], &setpoint[3], kP[3], kI, kD[3], DIRECT);

int pos_i[4];
unsigned long previousTime[4];
int prevPos[4];
double rpmSum[4];
double finalRpm[4];
double demand;
int count = 0;

unsigned long currentMillis;
unsigned long previousMillis;


void setup() {
  // put your setup code here, to run once:
  pinMode(ENA_1, OUTPUT);
  pinMode(ENB_1, OUTPUT);
  pinMode(ENA_2, OUTPUT);
  pinMode(ENB_2, OUTPUT);

  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  
  pinMode(ENCA_1, INPUT);
  pinMode(ENCB_1, INPUT);
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_2, INPUT);
  pinMode(ENCA_3, INPUT);
  pinMode(ENCB_3, INPUT);
  pinMode(ENCA_4, INPUT);
  pinMode(ENCB_4, INPUT);
  pidController2.SetMode(AUTOMATIC);
  pidController3.SetMode(AUTOMATIC);
  pidController4.SetMode(AUTOMATIC);

  pinMode(ENCPOWER, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCB_1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_4), readEncoder4, RISING);

  pidController1.SetMode(AUTOMATIC);

  pidController1.SetSampleTime(10);
  pidController2.SetSampleTime(10);
  pidController3.SetSampleTime(10);
  pidController4.SetSampleTime(10);

  //Serial.begin(9600);
  digitalWrite(ENCPOWER, HIGH);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(calcRPM);
  demand = 0.0;
}


void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    if (Serial.available() > 0) {
      Serial.println("changing speed");
      int n = Serial.parseInt();
      demand = (double)n / 10;
    } else {
       demand = demand;  
    }
    Serial.println(demand);
    mecanumDrive(demand, 0, 0);
  } 
  /*  
  for(int i = 0; i < 4; i++) {
    Serial.print(finalRpm[i]);
    Serial.print(" ");
  }
  Serial.println();
  */
}

 
void calcRPM() {
  count++;
  for(int i = 0; i < 4; i++) {
    int deltaPos = pos_i[i] - prevPos[i];
    double revs = deltaPos / 3000.0;
    double rpmCalc = revs * 60000;
    prevPos[i] = pos_i[i];
    rpmSum[i] += rpmCalc;
    if (count >= 10) {
      finalRpm[i] = rpmSum[i] / 10.0;
      rpmSum[i] = 0;
    }
  }
  if(count >= 10) {
    count = 0;
  }
}

void mecanumDrive(float x, float y, float rotation) {
  double frontLeft = y + x + rotation;
  double frontRight = y - x - rotation;
  double backLeft = y - x + rotation;
  double backRight = y + x - rotation;

  double frontLeftRPM = (frontLeft) * 340.0 / 2.0;
  double frontRightRPM = frontRight * 340.0 / 2.0;
  double backLeftRPM = backLeft * 340.0 / 2.0;
  double backRightRPM = backRight * 340.0 / 2.0;

  double fl = abs(frontLeft) * (255 - 50) + 50;
  double fr = abs(frontRight) * (255-50) + 50;
  double bl = abs(backLeft) * (255-50) + 50;
  double br = abs(backRight) * (255-50) + 50;
  
  setpoint[0] = abs(frontLeftRPM);
  setpoint[1] = abs(frontRightRPM);
  setpoint[2] = abs(backLeftRPM);
  setpoint[3] = abs(backRightRPM);
  
  for(int i = 0; i < 4; i++) {
    in[i] = abs(finalRpm[i]);  
  }
  pidController1.Compute();
  pidController2.Compute();
  pidController3.Compute();
  pidController4.Compute();
  
  frontLeft = constrain(frontLeft * 255, -255, 255);
  frontRight = frontRight * 255;
  backLeft = backLeft * 255;
  backRight = backRight * 255;

  // Apply the calculated values to the motor control pins
  if(frontLeft >= 0) {
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN2_1, HIGH);
  } else {
    digitalWrite(IN1_1, HIGH);
    digitalWrite(IN2_1, LOW);
  }
  analogWriteFrequency(ENA_1, 490);
  analogWrite(ENA_1, fl);
  //analogWrite(ENA_1, 25);
  
  if(frontRight >= 0) {
    digitalWrite(IN3_1, HIGH);
    digitalWrite(IN4_1, LOW);
  } else {
    digitalWrite(IN3_1, LOW);
    digitalWrite(IN4_1, HIGH);
  }
  analogWriteFrequency(ENB_1, 490);
  analogWrite(ENB_1, fr);
  //analogWrite(ENB_1, 25);


  if(backLeft >= 0) {
    digitalWrite(IN3_2, LOW);
    digitalWrite(IN4_2, HIGH);
  } else {
    digitalWrite(IN3_2, HIGH);
    digitalWrite(IN4_2, LOW);
  }
  analogWriteFrequency(ENB_2, 490);
  analogWrite(ENB_2, bl);
  //analogWrite(ENB_2, 25);

  if(backRight >= 0) {
    digitalWrite(IN1_2, LOW);
    digitalWrite(IN2_2, HIGH);
  } else {
    digitalWrite(IN1_2, HIGH);
    digitalWrite(IN2_2, LOW);
  }
  analogWriteFrequency(ENA_2, 490);
  analogWrite(ENA_2, br);
  //analogWrite(ENA_2, 25);

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
