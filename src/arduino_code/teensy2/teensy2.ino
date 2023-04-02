#include "amc.h"
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/String.h"
#include "Adafruit_VL6180X.h"

#define UPPER_LIMIT 39
#define LOWER_LIMIT 38

//Setting up the ros node for wheel commands
// Setting up the ros node for wheel commands
ros::NodeHandle nh;
//Thie type of message is geometry_msgs::Twist
// Thie type of message is geometry_msgs::Twist

Amc* arm;


//std_msgs::Int32MultiArray carouselMsg;
std_msgs::UInt16MultiArray pedestalColorMsg;

std_msgs::Int16 gripperRotateCmd;
std_msgs::Int16 gripperClampCmd;
std_msgs::Int16 wristCmd;
std_msgs::Int16 doorCmd;
std_msgs::Int16 paddleCmd;
std_msgs::Int16 armCmd;
std_msgs::Int16 foodChipColorCmd;



//Stepper ros variables
std_msgs::Int16 carouselCmd;
std_msgs::Int16 liftingCmd;


ros::Publisher PedestalColor("/bot/pedestalColor", &pedestalColorMsg);


void gripperRotateCB(const std_msgs::Int16 &cmd_msg)
{
  gripperRotateCmd.data = cmd_msg.data;
}

void gripperClampCB(const std_msgs::Int16 &cmd_msg)
{
  gripperClampCmd.data = cmd_msg.data;
}


void doorCB(const std_msgs::Int16 &cmd_msg)
{
  doorCmd.data = cmd_msg.data;
}

void armCB(const std_msgs::Int16 &cmd_msg)
{
  armCmd.data = cmd_msg.data;
}

void wristCB(const std_msgs::Int16 &cmd_msg)
{
  wristCmd.data = cmd_msg.data;
}


void paddleCB(const std_msgs::Int16 &cmd_msg)
{
  paddleCmd.data = cmd_msg.data;
}

//This will be the callback function for wheel commands from jetson

void foodChipColorCB(const std_msgs::Int16 &cmd_msg)
{
  foodChipColorCmd.data = cmd_msg.data;
}


void carouselCB(const std_msgs::Int16 &cmd_msg)
{
  carouselCmd.data = cmd_msg.data;
}


void liftingCB(const std_msgs::Int16 &cmd_msg)
{
  liftingCmd.data = cmd_msg.data;
}



ros::Subscriber<std_msgs::Int16> gripperRotate("/bot/gripperRotate_cmd", gripperRotateCB);
ros::Subscriber<std_msgs::Int16> gripperClamp("/bot/gripperClamp_cmd", gripperClampCB);
ros::Subscriber<std_msgs::Int16> door("/bot/door_cmd", doorCB);
ros::Subscriber<std_msgs::Int16> elbow("/bot/arm_cmd", armCB);
ros::Subscriber<std_msgs::Int16> wrist("/bot/wrist_cmd", wristCB);
ros::Subscriber<std_msgs::Int16> paddle("/bot/paddle_cmd", paddleCB);
ros::Subscriber<std_msgs::Int16> foodChipColor("/bot/foodChipColor", foodChipColorCB);


ros::Subscriber<std_msgs::Int16> carousel("/bot/carousel_cmd", carouselCB);
ros::Subscriber<std_msgs::Int16> lifting("/bot/lifting_cmd", liftingCB);


// This will be the callback function for wheel commands from jetson
Adafruit_VL6180X tofSensor = Adafruit_VL6180X();
std_msgs::Float32 tofMsg;
ros::Publisher tofPub("/bot/tof", &tofMsg);

void setup()
{
    arm = new Amc();
    setupTOF();
    //Wire2.begin();

    nh.initNode();
    
    nh.subscribe(gripperRotate);
    nh.subscribe(gripperClamp);
    nh.subscribe(door);
    nh.subscribe(elbow);
    nh.subscribe(wrist);
    nh.subscribe(paddle);
    nh.subscribe(foodChipColor);


    nh.subscribe(carousel);
    nh.subscribe(lifting);

    nh.advertise(PedestalColor);
    nh.advertise(tofPub);

    gripperRotateCmd.data = 0;
    gripperClampCmd.data = 0;
    wristCmd.data = 180;
    armCmd.data = 172;
    paddleCmd.data = 135;
    doorCmd.data = 120;
    foodChipColorCmd.data = 90;

    liftingCmd.data = 0;
    carouselCmd.data = 0;

    
    pedestalColorMsg.data_length = 4;
    pedestalColorMsg.data = (uint16_t *)malloc(4 * sizeof(uint16_t));

    for (int i = 0; i < 4; i++)
     {
        pedestalColorMsg.data[i] = uint16_t(0);
      }

}

void loop()
{
    readTOF();
    tofPub.publish(&tofMsg);
    
    
    arm->gripperRotateCmd(gripperRotateCmd.data);
    

    arm->doorCmd(doorCmd.data);
    

    arm->armCmd(armCmd.data);
   

    arm->paddleCmd(paddleCmd.data);
    

    arm->gripperClampCmd(gripperClampCmd.data);

    arm->wristCmd(wristCmd.data);

    arm->foodChipCmd(foodChipColorCmd.data);


    arm->liftingCmd(liftingCmd.data);
    
    if (liftingCmd.data == 1 && digitalRead(UPPER_LIMIT)== LOW) {liftingCmd.data = 0;}
    if (liftingCmd.data == -1 && digitalRead(LOWER_LIMIT)== LOW) {liftingCmd.data = 0;}
    arm->liftingCmd(liftingCmd.data);
   
    
    arm->carouselCmd(carouselCmd.data);
    
    //Reset the message after pedestal loaded
    carouselCmd.data = 0;

    
    pedestalColorMsg.data[0] = arm->r;
    pedestalColorMsg.data[1] = arm->g;
    pedestalColorMsg.data[2] = arm->b;
    pedestalColorMsg.data[3] = arm->c;

    PedestalColor.publish(&pedestalColorMsg);
    
    
    nh.spinOnce();
    //arm->getColorData();
    //Serial.println("END OF LOOP");
    delay(50);
}

void setupTOF() {
  if (!tofSensor.begin(&Wire1)) {
    Serial.println("Cannot find sensor");
    while (1);
  }
  Serial.println("Found TOF sensor");
}

void readTOF() {
  float lux = tofSensor.readLux(VL6180X_ALS_GAIN_5);
  float range = tofSensor.readRange();
  uint8_t status = tofSensor.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    tofMsg.data = range * 0.1;
  }
}
