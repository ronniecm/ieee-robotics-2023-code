#include "Arduino.h"
#include "MovingAverageFilter.hpp"
#include "ros.h"
#include "std_msgs/Float32.h"

class Ultrasonic
{
public:
    Ultrasonic();
    ~Ultrasonic();
//Distance measurements
    float getUltra0_Distance();
    float getUltra1_Distance();
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
    std_msgs::Float32 ultraMessages[8];
    ros::Publisher* ultraPublishers[8];
    float duration_Ultra0, cm_Ultra0;
    float duration_Ultra1, cm_Ultra1;
    float duration_Ultra2, cm_Ultra2;
    float duration_Ultra3, cm_Ultra3;
    float duration_Ultra4, cm_Ultra4;
    float duration_Ultra5, cm_Ultra5;
    float duration_Ultra6, cm_Ultra6;
    float duration_Ultra7, cm_Ultra7;
    MovingAverageFilter* filters[8];
    float readings[8];
};
