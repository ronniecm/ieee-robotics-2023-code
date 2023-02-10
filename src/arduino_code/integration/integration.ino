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

void setup()
{
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
    /*
    // Set your setpoint for x y rotation axes
    xSetpoint = 0.0;
    ySetpoint = 0.0;
    zSetpoint = 0.0;
    */
   xIn = ultrasonicSensors->getTopLeftDistance();
   yIn = ultrasonicSensors->getFrontLeftDistance();
   rotIn = ultrasonicSensors->getTopLeftDistance() - ultrasonicSensors->getBottomLeftDistance();

   xController.Compute();
   yController.Compute();
   rotController.Compute();

   drivetrain->mecanumDrive(xOut, yOut, rotOut);
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
