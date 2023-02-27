#include "Drivetrain.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"

//Setting up the ros node for wheel commands
ros::NodeHandle nh;
//Thie type of message is geometry_msgs::Twist
geometry_msgs::Twist msg;


#define FL_in1 2
#define FL_in2 3

#define FR_in1 36
#define FR_in2 37

#define BL_in1 22
#define BL_in2 23

#define BR_in1 28
#define BR_in2 29

Drivetrain *drivetrain;


//These will be the inputs to the mecanumDrive function
float cmd_x;
float cmd_y;
float cmd_z;

//This will be the callback function for wheel commands from jetson
void mecanumDriveCallBack(const geometry_msgs::Twist& cmd_msg)
{
  cmd_x = cmd_msg.linear.x;
  cmd_y = cmd_msg.linear.y;
  cmd_z = cmd_msg.angular.z;
  
}

ros::Subscriber <geometry_msgs::Twist> sub("/bot/cmd_vel", mecanumDriveCallBack);

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
    //Ros setup node & subscribe to topic
    nh.initNode();
    nh.subscribe(sub);
    
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
     //Mecanum drive now a function of twist msgs
     drivetrain->mecanumDrive(cmd_y, cmd_x,cmd_z);
     for(int i = 0; i < 4; i++) {
        Serial.print(drivetrain->getRPM(i));
        Serial.print(" ");
     }
     Serial.println();
   }

   nh.spinOnce();
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
