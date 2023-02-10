#include "Ultrasonic.h"

#define Front_Left_trigPin 4    // Trigger
#define Front_Left_echoPin 5    // Echo

//Back Left trig and echo pins and distance variables
#define Back_Left_trigPin 3
#define Back_Left_echoPin 2

//Top Left trig and echo pins and distance variables
#define Top_Left_trigPin 6
#define Top_Left_echoPin 7

//Top Right trig and echo pins and distance variables
#define Top_Right_trigPin 22
#define Top_Right_echoPin 23

//Front Right trig and echo pins and distance variables
#define Front_Right_trigPin 9
#define Front_Right_echoPin 8

//Back Right trig and echo pins and distance variables
#define Back_Right_trigPin 11
#define Back_Right_echoPin 10

//Bottom Right trig and echo pins and distance variables
#define Bottom_Right_trigPin 13
#define Bottom_Right_echoPin 12

#define Bottom_Left_trigPin 24
#define Bottom_Left_echoPin 25

int Ultrasonic::getFrontLeftDistance()
{
    digitalWrite(Front_Left_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Front_Left_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Front_Left_trigPin, LOW);
    duration_Front_Left = pulseIn(Front_Left_echoPin, HIGH);
    // Convert the time into a distance
    inches_Front_Left = (duration_Front_Left / 2) / 74; // Divide by 74 or multiply by 0.0135
    return inches_Front_Left;
}

int Ultrasonic::getBackLeftDistance()
{
    digitalWrite(Back_Left_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Back_Left_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Back_Left_trigPin, LOW);
    duration_Back_Left = pulseIn(Back_Left_echoPin, HIGH);
    inches_Back_Left = (duration_Back_Left / 2) / 74; // Divide by 74 or multiply by 0.0135
    return inches_Back_Left;
}

int Ultrasonic::getTopLeftDistance()
{
    // below is Top Left calculations
    digitalWrite(Top_Left_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Top_Left_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Top_Left_trigPin, LOW);
    duration_Top_Left = pulseIn(Top_Left_echoPin, HIGH);
    inches_Top_Left = (duration_Top_Left / 2) / 74;
    return inches_Top_Left;
}

int Ultrasonic::getTopRightDistance()
{
    // below is Top Left calculations
    digitalWrite(Top_Right_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Top_Right_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Top_Right_trigPin, LOW);
    duration_Top_Right = pulseIn(Top_Right_echoPin, HIGH);
    inches_Top_Right = (duration_Top_Right / 2) / 74;
    return inches_Top_Right;
}

int Ultrasonic::getFrontRightDistance()
{
    digitalWrite(Front_Right_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Front_Right_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Front_Right_trigPin, LOW);
    duration_Front_Right = pulseIn(Front_Right_echoPin, HIGH);
    inches_Front_Right = (duration_Front_Right / 2) / 74;
    return inches_Front_Right;
}
int Ultrasonic::getBackRightDistance()
{
    digitalWrite(Back_Right_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Back_Right_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Back_Right_trigPin, LOW);
    duration_Back_Right = pulseIn(Back_Right_echoPin, HIGH);
    inches_Back_Right = (duration_Back_Right / 2) / 74;
    return inches_Back_Right;
}

int Ultrasonic::getBottomRightDistance()
{
    digitalWrite(Bottom_Right_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Bottom_Right_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Bottom_Right_trigPin, LOW);
    duration_Bottom_Right = pulseIn(Bottom_Right_echoPin, HIGH);
    inches_Bottom_Right = (duration_Bottom_Right / 2) / 74;
    return inches_Bottom_Right;
}

int Ultrasonic::getBottomLeftDistance()
{
    digitalWrite(Bottom_Left_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(Bottom_Left_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Bottom_Left_trigPin, LOW);
    duration_Bottom_Left = pulseIn(Bottom_Left_echoPin, HIGH);
    inches_Bottom_Left = (duration_Bottom_Left / 2) / 74;
    return inches_Bottom_Left;
}