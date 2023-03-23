#include "Drivetrain.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "amc.h"
#include "TeensyThreads.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/String.h"

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

std_msgs::Int32MultiArray carouselMsg;
std_msgs::UInt16MultiArray pedestalColorMsg;
std_msgs::Int16 cmd_msg;
std_msgs::String foodChipColorMsg;


std_msgs::Int16 gripperRotateCmd;
std_msgs::Int16 gripperClampCmd;
std_msgs::Int16 doorCmd;
std_msgs::Int16 armCmd;
std_msgs::Int16 wristCmd;
std_msgs::Int16 paddleCmd;
std_msgs::Int16 liftingCmd;
std_msgs::Int16 carouselCmd;
std_msgs::Int16 foodChipColorCmd;

//These will be the inputs to the mecanumDrive function
float cmd_x;
float cmd_y;
float cmd_z;

ros::Publisher Carousel("/bot/carousel_callback", &carouselMsg);
ros::Publisher PedestalColor("/bot/pedestalColor", &pedestalColorMsg);
ros::Publisher foodChip("/bot/foodChipsTATUS", &foodChipColorMsg);

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

void foodChipColorCB(const std_msgs::Int16& cmd_msg) {
  foodChipColorCmd.data = cmd_msg.data;
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
ros::Subscriber <std_msgs::Int16> foodChipColor("/bot/foodChipColor", &foodChipColorCB);

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
    initDrivetrain();
    Serial.println("init drivetrain");
    drivetrain = new Drivetrain();
    Serial.println("constructed drivetrain");
    arm = new Amc();
    Serial.println("constructed arm");

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
    nh.subscribe(pid);
    nh.advertise(Carousel);
    nh.advertise(PedestalColor);
    nh.subscribe(foodChipColor);
    nh.advertise(foodChip);
    
    gripperRotateCmd.data = 90;
    gripperClampCmd.data = 0;
    wristCmd.data = 180;
    armCmd.data = 180;
    paddleCmd.data = 180;
    doorCmd.data = 120;
    carouselCmd.data = 0;
    liftingCmd.data = 1;
    
    carouselMsg.data_length = 5;
    carouselMsg.data = (int32_t*) malloc(5 * sizeof(int32_t));

    for(int i=0;i<5;i++){
      carouselMsg.data[i] = int32_t(0);
    }

    pedestalColorMsg.data_length = 4;
    pedestalColorMsg.data = (uint16_t*) malloc(4 * sizeof(uint16_t));

    for(int i=0;i<4;i++){
      pedestalColorMsg.data[i] = uint16_t(0);
    }
     
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
     //Mecanum drive now a function of twist msgs
      drivetrain->mecanumDrive(0.0, demand,0.0);
      for(int i = 0; i < 4; i++) {
        Serial.print(drivetrain->getRPM(i));
        Serial.print(" ");
      }
      Serial.println();
    }
    /*
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

    //arm->foodChipCmd(foodChipColorCmd.data);
    //foodChip.publish(foodChipColorCmd.data);

    arm->liftingCmd(liftingCmd.data);
    if (liftingCmd.data == 1 && digitalRead(UPPER_LIMIT)== LOW) {liftingCmd.data = 0;}
    if (liftingCmd.data == -1 && digitalRead(LOWER_LIMIT)== LOW) {liftingCmd.data = 0;}
    
    arm->liftingCmd(liftingCmd.data);

    
    arm->carouselCmd(carouselCmd.data);
    if(arm->getStepsLeft() == 0) {
      carouselCmd.data = 0;
      arm->carouselCmd(carouselCmd.data);  
    }
    
   }

   for(int i = 0; i < 5; i++){
    carouselMsg.data[i] = int32_t(arm->slots[i]);
   }
   

   Carousel.publish(&carouselMsg);
   
   /*
   for(int i = 0; i < 4; i++){
    Serial.println(pedestalColorMsg.data[i]);
   }
   pedestalColorMsg.data[0] = arm->r;
   pedestalColorMsg.data[1] = arm->g;
   pedestalColorMsg.data[2] = arm->b;
   pedestalColorMsg.data[3] = arm->c;
   

   PedestalColor.publish(&pedestalColorMsg);
  
  
  if (foodChipColorCmd.data == 1) {
    foodChipColorMsg.data = "detecting";
    foodChip.publish(&foodChipColorMsg);
    foodChipColorCmd.data = 0;
    int color = arm->getFoodChipColor(); 
    if (color == 0) {
      foodChipColorMsg.data = "detected red";
    } else {
      foodChipColorMsg.data = "detected green";
    }
    foodChip.publish(&foodChipColorMsg);
    arm->foodChipCmd(90);
  }
  

  delay(10);
  nh.spinOnce();
  
   */
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

    Timer1.initialize(800);
    Timer1.attachInterrupt(calcRPM);
}

void calcRPM()
{
    drivetrain->calcRPM();
}
