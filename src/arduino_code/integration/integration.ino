#include "Drivetrain.h"
#include "Ultrasonic.h"

#define FL_in1 14
#define FL_in2 15

#define FR_in1 36
#define FR_in2 37

#define BL_in1 5
#define BL_in2 4

#define BR_in1 24
#define BR_in2 6

Drivetrain *drivetrain;
Ultrasonic *ultrasonicSensors;

double xP, xI, xD;
double xIn;
double xOut;
double xSetpoint;
PID xController(&xIn, &xOut, &xOut, xP, xI, xD, DIRECT);

double yP, yI, yD;
double yIn;
double yOut;
double ySetpoint;
PID yController(&yIn, &yOut, &yOut, yP, yI, yD, DIRECT);

double rotP, rotI, rotD;
double rotIn;
double rotOut;
double rotSetpoint;

PID rotController(&rotIn, &rotOut, &rotOut, rotP, rotI, rotD, DIRECT);

unsigned long currentMillis;
unsigned long previousMillis;

double demand;

void setup()
{
    drivetrain = new Drivetrain();

    initDrivetrain();
    
    xController.SetMode(AUTOMATIC);
    xController.SetOutputLimits(-1.0, 1.0);
    xController.SetSampleTime(10);

    yController.SetMode(AUTOMATIC);
    yController.SetOutputLimits(-1.0, 1.0);
    yController.SetSampleTime(10);

    yController.SetMode(AUTOMATIC);
    yController.SetOutputLimits(-1.0, 1.0);
    yController.SetSampleTime(10);
}

void loop()
{
   currentMillis = millis();
   while(currentMillis - previousMillis >= 10) {
     previousMillis = currentMillis;
     if (Serial.available() > 0) {
        int n = Serial.parseInt();
        demand = (double) n / 10.0; 
     }
     drivetrain->mecanumDrive(0,demand,0.0);
     for(int i = 0; i < 4; i++) {
        Serial.print(drivetrain->getRPM(i));
        Serial.print(" ");
     }
     Serial.println();
   }
}

void initDrivetrain()
{
    pinMode(FL_in1, OUTPUT);
    pinMode(FL_in2, OUTPUT);
    pinMode(FR_in1, OUTPUT);
    pinMode(FR_in2, OUTPUT);
    pinMode(BL_in1, OUTPUT);
    pinMode(BL_in2, OUTPUT);
    pinMode(BR_in1, OUTPUT);
    pinMode(BR_in2, OUTPUT);

    Timer1.initialize(1000);
    Timer1.attachInterrupt(calcRPM);
}

void calcRPM()
{
    drivetrain->calcRPM();
}
