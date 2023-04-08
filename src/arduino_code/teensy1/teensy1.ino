#include "Drivetrain.h"
#include "IntervalTimer.h"
#include "Ultrasonic.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <Wire.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "Adafruit_TCS34725.h"


#define BLACK 0
#define WHITE 1
#define GREEN 2
#define RED 3


Drivetrain* drivetrain;
Ultrasonic* ultrasonics;


std_msgs::Int16 sendColorValue ;


std_msgs::UInt16MultiArray colorValuesMsg;

std_msgs::UInt16MultiArray rgb1Msg;

std_msgs::UInt16MultiArray rgb2Msg;


IntervalTimer intTimer;

Adafruit_TCS34725 foodChipsTcs1(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 foodChipsTcs2(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

uint16_t r1, g1, b1, c1, lux1;
uint16_t r2, g2, b2, c2, lux2;

float redCentroid[3] = {417.8, 151.6, 141.4};
float greenCentroid[3] = {278.6, 242.2, 166};
float ringCentroid[3] = {465.5, 366, 321};
float blueCentroid[3] = {274.2, 217.6, 269.8};



ros::NodeHandle nh;


//This will probabbly be a array of 2
ros::Publisher colorValues("/bot/colorValues", &colorValuesMsg);
ros::Publisher rgbValues1("/bot/rgb1", &rgb1Msg);
ros::Publisher rgbValues2("/bot/rgb2", &rgb2Msg);

void getColorValue(const std_msgs::Int16& msg) {
   sendColorValue.data = msg.data;
}

ros::Subscriber<std_msgs::Int16> colorRequest("/bot/colorRequest", getColorValue);

float cmd_x, cmd_y, cmd_z;
void mecanumCallback(const geometry_msgs::Twist& msg) {
  cmd_x = msg.linear.x;
  cmd_y = msg.linear.y;
  cmd_z = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> speedSub("/bot/cmd_vel", mecanumCallback);


unsigned long currentMillis, previousMillis;

void setup()
{
    //Serial.begin(115200);
    nh.initNode();
    sendColorValue.data = 0;
    drivetrain = new Drivetrain();
    initDrivetrain();
    ultrasonics = new Ultrasonic();
    initUltrasonics();
    for(int i = 0; i < 6; i++) {
      nh.advertise(*ultrasonics->getPub(i));
    }


    rgb1Msg.data_length = 4;
    rgb1Msg.data = (uint16_t *)malloc(4 * sizeof(uint16_t));
    for (int i = 0; i < 4; i++)
     {
        rgb1Msg.data[i] = uint16_t(0);
      }

    rgb2Msg.data_length = 4;
    rgb2Msg.data = (uint16_t *)malloc(4 * sizeof(uint16_t));
    for (int i = 0; i < 4; i++)
     {
        rgb2Msg.data[i] = uint16_t(0);
      }
      
    colorValuesMsg.data_length = 2;
    colorValuesMsg.data = (uint16_t *)malloc(2 * sizeof(uint16_t));
    for (int i = 0; i < 2; i++)
     {
        colorValuesMsg.data[i] = uint16_t(0);
      }
    
    nh.advertise(colorValues);
    nh.advertise(rgbValues1);
    nh.advertise(rgbValues2);

    
    nh.subscribe(colorRequest);
    nh.subscribe(speedSub); 
    
    foodChipsTcs1.begin();
    foodChipsTcs2.begin(0x29, &Wire1);
       
}

void loop()
{
  //cmd_x = 1.0;
  //cmd_y = 0.0;
  //cmd_z = 0.0;
  
  // Mecanum drive now a function of twist msgs
  currentMillis = millis();
  while (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    drivetrain->mecanumDrive(cmd_y, cmd_x,  cmd_z);
    //drivetrain->mecanumDrive(0.0, 0.80, 0.0);
  }
  ultrasonics->publishData();
  
  if(sendColorValue.data == 1){
    //Make a function for getting Color
    foodChipsTcs1.getRawData(&r1, &g1, &b1, &c1); 
    foodChipsTcs2.getRawData(&r2, &g2, &b2, &c2); 

    colorValuesMsg.data[0] = calc1Color(r1, g1, b1, c1);
    colorValuesMsg.data[1] = calc2Color(r2, g2, b2, c2);
    sendColorValue.data = 0;
    //Finally we can publish the msg

    rgb1Msg.data[0] = r1;
    rgb1Msg.data[1] = g1;
    rgb1Msg.data[2] = b1;
    rgb1Msg.data[3] = c1;

    rgb2Msg.data[0] = r2;
    rgb2Msg.data[1] = g2;
    rgb2Msg.data[2] = b2;
    rgb2Msg.data[3] = c2;

   
    colorValues.publish(&colorValuesMsg);   
    rgbValues1.publish(&rgb1Msg);   
    rgbValues2.publish(&rgb2Msg);   
    
  }
  
  //Serial.println(r1);
  
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

  intTimer.begin(calcRPM, 800);
}

void calcRPM()
{
  drivetrain->calcRPM();
}

void initUltrasonics() {
  pinMode(Ultra2_trigPin,OUTPUT);
  pinMode(Ultra2_echoPin,INPUT);
  
  pinMode(Ultra3_trigPin,OUTPUT);
  pinMode(Ultra3_echoPin,INPUT);

  pinMode(Ultra4_trigPin,OUTPUT);
  pinMode(Ultra4_echoPin,INPUT);

  pinMode(Ultra5_trigPin,OUTPUT);
  pinMode(Ultra5_echoPin,INPUT);

  pinMode(Ultra6_trigPin,OUTPUT);
  pinMode(Ultra6_echoPin,INPUT);

  pinMode(Ultra7_trigPin,OUTPUT);
  pinMode(Ultra7_echoPin,INPUT);
}

//Right Sensor If looking foward
int calc1Color(int r, int g, int b, int c){
  //Determine color based on rgb values

  if (r > 415 && r < 425 && g > 145 && g < 155 && b > 135 && b < 145)
      {
        return RED;
      }
      else if (r > 275 && r < 285 && g > 245 && g < 255 && b > 165 && b < 175)
      {
        return GREEN;
      }
      else if (c >240)
      {
        return WHITE;
      }
      else{
        return BLACK;
      }
}

//Left Sensor If looking foward
int calc2Color(int r, int g, int b, int c){
  //Determine color based on rgb values

  if (r > 85 && r < 95 && g > 75 && g < 85 && b > 50 && b < 60)
      {
        return GREEN;
      }
      else if (r > 130 && r < 140 && g > 45 && g < 50 && b > 40 && b < 50)
      {
        return RED;
        
      }
      else if (c >230)
      {
        return WHITE;
      }
      else{
        return BLACK;
      }
}


int determineColor(float r, float g, float b) {
  int color = -1;
  float p[3] = {r, g, b};
  float minDistance = 10000;

  float dist = calcDistance(p, redCentroid);
  if (dist < minDistance) {
    minDistance = min(dist, minDistance);
    color = 0;
  }
  
  dist = calcDistance(p, greenCentroid);
  if (dist < minDistance) {
    minDistance = min(dist, minDistance);
    color = 1;
  }
  
  dist = calcDistance(p, ringCentroid);
  if (dist < minDistance) {
    minDistance = min(dist, minDistance);
    color = 2;
  }
  
  dist = calcDistance(p, blueCentroid);
  if (dist < minDistance) {
    minDistance = min(dist, minDistance);
    color = 3;
  }

  return color;
}

float calcDistance(float* p1, float* p2) {
    float dist = sqrt( pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2) );
    return dist;
}
