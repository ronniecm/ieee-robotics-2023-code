
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "Ultrasonic.h"


float inches, duration;

//ROS TOPICS:
//'/bot/ultraFront'
//'/bot/ultraRight'
//'/bot/ultraBack'
//'/bot/ultraLeft'
//Each topic includes a tuple of the sensor data for each side of the bot


ros::NodeHandle nh;
//Initalize the message type for each topic
std_msgs::Float32MultiArray distMsgFront;
std_msgs::Float32MultiArray distMsgRight;
std_msgs::Float32MultiArray distMsgBack;
std_msgs::Float32MultiArray distMsgLeft;

ros::Publisher front("/bot/ultraFront", &distMsgFront);
ros::Publisher right("/bot/ultraRight", &distMsgRight);
ros::Publisher back("/bot/ultraBack", &distMsgBack);
ros::Publisher left("/bot/ultraLeft", &distMsgLeft);

Ultrasonic ultraSensors;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Time to init the sensor node
  nh.initNode();
  
  distMsgFront.data_length = 2;
  distMsgRight.data_length = 2;
  distMsgBack.data_length = 2;
  distMsgLeft.data_length = 2;
  
  //This will initialize the array of Float32MultiArray for the tuple message that will be
  //published to the respective topic
  distMsgFront.data = (float *)malloc((sizeof(float))*distMsgFront.data_length*2);
  distMsgRight.data = (float *)malloc((sizeof(float))*distMsgRight.data_length*2);
  distMsgBack.data = (float *)malloc((sizeof(float))*distMsgBack.data_length*2);
  distMsgLeft.data = (float *)malloc((sizeof(float))*distMsgLeft.data_length*2);

  //This make the information on the topic available to subscribers
  nh.advertise(front);
  nh.advertise(right);
  nh.advertise(back);
  nh.advertise(left);

}
 
void loop() {
    //We will save corresponding sensor data to the tuple msg
    
    // --------------------IMPORTANT------------------------
    //TO KEEP THINGS CONSISTANT 0 index ==> LEFT SENSOR & 1 index ==> RIGHT for each side
    
    distMsgFront.data[0] = ultraSensors.getFrontLeftDistance();
    distMsgFront.data[1] = ultraSensors.getFrontRightDistance();
   
    distMsgRight.data[0] = ultraSensors.getTopRightDistance();
    distMsgRight.data[1] = ultraSensors.getBottomRightDistance();

    distMsgBack.data[0] = ultraSensors.getBackRightDistance();
    distMsgBack.data[1] = ultraSensors.getBackLeftDistance();
    
    distMsgLeft.data[0] = ultraSensors.getBottomLeftDistance();
    distMsgLeft.data[1] = ultraSensors.getTopLeftDistance();

    front.publish(&distMsgFront);
    right.publish(&distMsgRight);
    back.publish(&distMsgBack);
    left.publish(&distMsgLeft);
    nh.spinOnce();
    
    
    


}
