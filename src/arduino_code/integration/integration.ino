#include "Drivetrain.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "amc.h"
#include "TeensyThreads.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"

#define FL_in1 2
#define FL_in2 3

#define FR_in1 36
#define FR_in2 37

#define BL_in1 22
#define BL_in2 23

#define BR_in1 28
#define BR_in2 29

#define UPPER_LIMIT 39
#define LOWER_LIMIT 38

//Setting up the ros node for wheel commands
ros::NodeHandle nh;
//Thie type of message is geometry_msgs::Twist
geometry_msgs::Twist msg;

double demand;
double kP, kI, kD;
Drivetrain *drivetrain;
Amc* arm;

std_msgs::Int16 gripperRotateMsg;
std_msgs::Int16 gripperClampMsg;
std_msgs::Int16 doorMsg;
std_msgs::Int16 armMsg;
std_msgs::Int16 wristMsg;
std_msgs::Int16 paddleMsg;
std_msgs::Int16 liftingMsg;
std_msgs::Int16 carouselMsg;
std_msgs::Int16 cmd_msg;

std_msgs::Int16 gripperRotateCmd;
std_msgs::Int16 gripperClampCmd;
std_msgs::Int16 doorCmd;
std_msgs::Int16 armCmd;
std_msgs::Int16 wristCmd;
std_msgs::Int16 paddleCmd;
std_msgs::Int16 liftingCmd;
std_msgs::Int16 carouselCmd;

//These will be the inputs to the mecanumDrive function
float cmd_x;
float cmd_y;
float cmd_z;

ros::Publisher GripperRotate("/bot/gripperRotate_callback", &gripperRotateMsg);
ros::Publisher GripperClamp("/bot/gripperClamp_callback", &gripperClampMsg);
ros::Publisher Door("/bot/door_callback", &doorMsg);
ros::Publisher Arm("/bot/arm_callback", &armMsg);
ros::Publisher Wrist("/bot/wrist_callback", &wristMsg);
ros::Publisher Paddle("/bot/paddle_callback", &paddleMsg);
ros::Publisher Lifting("/bot/lifting_callback", &liftingMsg);
ros::Publisher Carousel("/bot/carousel_callback", &carouselMsg);


void gripperRotateCB(const std_msgs::Int16& cmd_msg)
{
  gripperRotateCmd.data = cmd_msg.data;
}

void gripperClampCB(const std_msgs::Int16& cmd_msg)
{
  gripperClampCmd.data = cmd_msg.data;
}

void doorCB(const std_msgs::Int16& cmd_msg)
{
  doorCmd.data = cmd_msg.data;
}

void armCB(const std_msgs::Int16& cmd_msg)
{
  armCmd.data = cmd_msg.data;
}

void wristCB(const std_msgs::Int16& cmd_msg)
{
  wristCmd.data = cmd_msg.data;
}

void paddleCB(const std_msgs::Int16& cmd_msg)
{
  paddleCmd.data = cmd_msg.data;
}

void liftingCB(const std_msgs::Int16& cmd_msg)
{
  liftingCmd.data = cmd_msg.data;
  liftingMsg.data = liftingCmd.data;
  Lifting.publish(&liftingMsg);
}

void carouselCB(const std_msgs::Int16& cmd_msg)
{
  carouselCmd.data = cmd_msg.data;
}

void pidCB(const std_msgs::Float32MultiArray& cmd_msg)
{
  kP = cmd_msg.data[0];
  kI = cmd_msg.data[1];
  kD = cmd_msg.data[2];

  //Now that we have the PID values, we can set them in the PID object
  drivetrain->tunePID(kP, kI, kD);
}

ros::Subscriber <std_msgs::Int16> gripperRotate("/bot/gripperRotate_cmd", &gripperRotateCB);
ros::Subscriber <std_msgs::Int16> gripperClamp("/bot/gripperClamp_cmd", &gripperClampCB);
ros::Subscriber <std_msgs::Int16> door("/bot/door_cmd", &doorCB);
ros::Subscriber <std_msgs::Int16> elbow("/bot/arm_cmd", &armCB);
ros::Subscriber <std_msgs::Int16> wrist("/bot/wrist_cmd", &wristCB);
ros::Subscriber <std_msgs::Int16> paddle("/bot/paddle_cmd", &paddleCB);
ros::Subscriber <std_msgs::Int16> lifting("/bot/lifting_cmd", &liftingCB);
ros::Subscriber <std_msgs::Int16> carousel("/bot/carousel_cmd", &carouselCB);
ros::Subscriber <std_msgs::Float32MultiArray> pid("/bot/PID", &pidCB);

//This will be the callback function for wheel commands from jetson
void mecanumDriveCallBack(const geometry_msgs::Twist& cmd_msg)
{
  cmd_x = cmd_msg.linear.x;
  cmd_y = cmd_msg.linear.y;
  cmd_z = cmd_msg.angular.z;
  
}

ros::Subscriber <geometry_msgs::Twist> sub("/bot/cmd_vel", mecanumDriveCallBack);

unsigned long currentMillis;
unsigned long previousMillis;


void setup()
{
    Serial.begin(115200);
    drivetrain = new Drivetrain();
    arm = new Amc();

    initDrivetrain();
    //Ros setup node & subscribe to topic
    nh.initNode();
    nh.subscribe(sub);    
    nh.subscribe(gripperRotate);
    nh.subscribe(gripperClamp);
    nh.subscribe(door);
    nh.subscribe(elbow);
    nh.subscribe(wrist);
    nh.subscribe(paddle);
    nh.subscribe(lifting);
    nh.subscribe(carousel);
    nh.advertise(Lifting);
    nh.subscribe(pid);

    gripperRotateCmd.data = 90;
    gripperClampCmd.data = 0;
    wristCmd.data = 180;
    armCmd.data = 180;
    paddleCmd.data = 180;
    doorCmd.data = 100;
    carouselCmd.data = 0;
    liftingCmd.data = 1;
       
    pinMode(33, OUTPUT);
    pinMode(38, INPUT);
    pinMode(39, INPUT);
}

void loop()
{
  
   //Mecanum drive now a function of twist msgs
   currentMillis = millis();
   while(currentMillis - previousMillis >= 10) {
      previousMillis = currentMillis;
       if (Serial.available() > 0) {
        int n = Serial.parseInt();
        demand = (double) n / 10.0; 
     }
     //Mecanum drive now a function of twist msgs
    drivetrain->mecanumDrive(cmd_y, cmd_x,cmd_z);
    
    arm->gripperRotateCmd(gripperRotateCmd.data);
    //GripperRotate.publish(&gripperRotateCmd);
    
    arm->gripperClampCmd(gripperClampCmd.data);
    //GripperClamp.publish(&gripperClampCmd);
  
    arm->doorCmd(doorCmd.data);
    //Door.publish(&doorCmd);
    
    arm->armCmd(armCmd.data);
    //Arm.publish(&armCmd);
    
    arm->wristCmd(wristCmd.data);
    //Wrist.publish(&wristCmd);
  
    arm->paddleCmd(paddleCmd.data);
    //Paddle.publish(&paddleCmd);

    arm->liftingCmd(liftingCmd.data);
    if (liftingCmd.data == 1 && digitalRead(UPPER_LIMIT)== LOW) {liftingCmd.data = 0;}
    if (liftingCmd.data == -1 && digitalRead(LOWER_LIMIT)== LOW) {liftingCmd.data = 0;}
    
    arm->liftingCmd(liftingCmd.data);

    
    arm->carouselCmd(carouselCmd.data);
    if(arm->getStepsLeft() == 0) {
      carouselCmd.data = 0;
      arm->carouselCmd(carouselCmd.data);  
    }
    //Carousel.publish(&carouselCmd);
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

    Timer1.initialize(800);
    Timer1.attachInterrupt(calcRPM);
}

void calcRPM()
{
    drivetrain->calcRPM();
}
