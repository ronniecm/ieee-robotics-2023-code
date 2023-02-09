#include <Encoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
#define FL_in1 14
#define FL_in2 15
Encoder FL_enc(22, 23);

#define FR_in1 36
#define FR_in2 37
Encoder FR_enc(34,33);

#define BL_in1 5
#define BL_in2 4
Encoder BL_enc(1,2);

#define BR_in1 24
#define BR_in2 6
Encoder BR_enc(32,31);

#define FL 0
#define FR 1
#define BL 2
#define BR 3

Encoder* enc[4];
int prevPos[4];
int count = 0;

double rpmSum[4];
double finalRpm[4];

double kP[4] = {8,8,8,8};
double kI = 0.0;
double kD[4] = {0,0,0,0};
double in[4];
double out[4];
double setpoint[4];

PID* speedController[4];

unsigned long currentMillis;
unsigned long previousMillis;

double demand = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(FL_in1, OUTPUT);
  pinMode(FL_in2, OUTPUT);
  pinMode(FR_in1, OUTPUT);
  pinMode(FR_in2, OUTPUT);
  pinMode(BL_in1, OUTPUT);
  pinMode(BL_in2, OUTPUT);
  pinMode(BR_in1, OUTPUT);
  pinMode(BR_in2, OUTPUT);

  
  enc[0] = new Encoder(22,23); 
  enc[1] = new Encoder(34,33); 
  enc[2] = new Encoder(1,2); 
  enc[3] = new Encoder(32,31);

  for(int i = 0; i < 4; i++) {
    speedController[i] = new PID(&in[i], &out[i], &setpoint[i], kP[i], kI, kD[i], DIRECT);  
    speedController[i]->SetMode(AUTOMATIC);
    speedController[i]->SetSampleTime(10);
  }
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(calcRPM);
}


void loop() {
  currentMillis = millis();
  while(currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    if(Serial.available() > 0) {
      int n = Serial.parseInt();
      demand = (double)n / 10.0;
    }
    mecanumDrive(0, demand, 0);
    ///analogWrite(BR_in1, 0);
    //analogWrite(BR_in2  , 255);
    for(int i = 0; i < 4; i++) {
      Serial.print(finalRpm[i]);
      Serial.print(" ");  
    }
    Serial.println();
  }
}


void calcRPM() {
  count++;
  for(int i = 0; i < 4; i++) {
    int currPos = enc[i]->read();
    int deltaPos = currPos - prevPos[i];
    double revs = deltaPos / 2678.0;
    double rpmCalc = revs * 60000;
    prevPos[i] = currPos;
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

  double frontLeftRPM = frontLeft * 200.0 / 2.0;
  double frontRightRPM = frontRight * 200.0 / 2.0;
  double backLeftRPM = backLeft * 200.0 / 2.0;
  double backRightRPM = backRight * 200.0 / 2.0;
  
  frontLeft = frontLeft  * 255;
  frontRight = frontRight * 255;
  backLeft = backLeft * 255;
  backRight = backRight * 255;

  setpoint[0] = abs(frontLeftRPM);
  setpoint[1] = abs(frontRightRPM);
  setpoint[2] = abs(backLeftRPM);
  setpoint[3] = abs(backRightRPM);

  for(int i = 0; i < 4; i++) {
    in[i] = abs(finalRpm[i]); 
    speedController[i]->Compute();
  }
  // Apply the calculated values to the motor control pins
  if(frontLeft >= 0) {
    analogWrite(FL_in1, 0);
    analogWrite(FL_in2, out[0]);
  } else {
    analogWrite(FL_in1, out[0]);
    analogWrite(FL_in2, 0);
  }
  if(frontRight >= 0) {
    analogWrite(FR_in1, out[1]);
    analogWrite(FR_in2, 0);
  } else {
    analogWrite(FR_in1, 0);
    analogWrite(FR_in2, out[1]);
  }
  if(backLeft >= 0) {
    analogWrite(BL_in1, out[2]);
    analogWrite(BL_in2, 0);
  } else {
    analogWrite(BL_in1, 0);
    analogWrite(BL_in2, out[2]);
  }
  if(backRight >= 0) {
    analogWrite(BR_in1, out[3]);
    analogWrite(BR_in2, 0);
  } else {
    analogWrite(BR_in1, 0);
    analogWrite(BR_in2, out[3]);
  }
}
