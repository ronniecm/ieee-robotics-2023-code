#include <stdint.h>
#include "Arduino.h"
#include "MovingAverageFilter.hpp"
#include "ros.h"
#include "std_msgs/Float32.h"

#define Ultra2_trigPin 23   // Trigger
#define Ultra2_echoPin 22 

#define Ultra3_trigPin 21
#define Ultra3_echoPin 20

#define Ultra4_trigPin 41
#define Ultra4_echoPin 40

#define Ultra5_trigPin 39
#define Ultra5_echoPin 38

#define Ultra6_trigPin 37
#define Ultra6_echoPin 36

#define Ultra7_trigPin 35
#define Ultra7_echoPin 34

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
    uint8_t trigPins[6] = {Ultra2_trigPin, Ultra3_trigPin, Ultra4_trigPin, Ultra5_trigPin, Ultra6_trigPin, Ultra7_trigPin};
    uint8_t echoPins[6] = {Ultra2_echoPin, Ultra3_echoPin, Ultra4_echoPin, Ultra5_echoPin, Ultra6_echoPin, Ultra7_echoPin};
    MovingAverageFilter* filters[6];
    float readings[6];
};
