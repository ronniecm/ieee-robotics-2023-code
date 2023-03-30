#include "Arduino.h"
#include "MovingAverageFilter.hpp"
#include "ros.h"
#include "std_msgs/Float32.h"

#define Ultra2_trigPin 23   // Trigger
#define Ultra2_echoPin 22 

#define Ultra3_trigPin 21
#define Ultra3_echoPin 20

#define Ultra4_trigPin 17
#define Ultra4_echoPin 16

#define Ultra5_trigPin 15
#define Ultra5_echoPin 14

#define Ultra6_trigPin 41
#define Ultra6_echoPin 40

#define Ultra7_trigPin 41
#define Ultra7_echoPin 40

class Ultrasonic
{
public:
    Ultrasonic();
    ~Ultrasonic();
//Distance measurements
    float getUltra2_Distance();
    float getUltra3_Distance();
    float getUltra4_Distance();
    float getUltra5_Distance();
    float getUltra6_Distance();
    float getUltra7_Distance();
    float getReading(int i);
    ros::Publisher* getPub(int i);
    void publishData();
    
private:
    std_msgs::Float32 ultraMessages[6];
    ros::Publisher* ultraPublishers[6];
    float duration_Ultra2, cm_Ultra2;
    float duration_Ultra3, cm_Ultra3;
    float duration_Ultra4, cm_Ultra4;
    float duration_Ultra5, cm_Ultra5;
    float duration_Ultra6, cm_Ultra6;
    float duration_Ultra7, cm_Ultra7;
    MovingAverageFilter* filters[6];
    float readings[6];
};
