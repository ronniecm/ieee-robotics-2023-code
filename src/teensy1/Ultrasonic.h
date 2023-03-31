#include <stdint.h>
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

#define Ultra7_trigPin 39
#define Ultra7_echoPin 38

class Ultrasonic
{
public:
    Ultrasonic();
    ~Ultrasonic();
//Distance measurements
    float getDistance(int i);
    float getReading(int i);
    ros::Publisher* getPub(int i);
    void publishData();
    
private:
    std_msgs::Float32 ultraMessages[6];
    ros::Publisher* ultraPublishers[6];
    uint8_t trigPins[6] = {23, 21, 17, 15, 41, 39};
    uint8_t echoPins[6] = {22, 20, 16, 14, 40, 38};
    MovingAverageFilter* filters[6];
    float readings[6];
};
