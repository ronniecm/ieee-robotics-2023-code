#include "Arduino.h"

class Ultrasonic
{
public:
    int getFrontLeftDistance();
    int getBackLeftDistance();
    int getTopLeftDistance();
    int getTopRightDistance();
    int getFrontRightDistance();
    int getBackRightDistance();
    int getBottomRightDistance();
    int getBottomLeftDistance();

private:
    int duration_Front_Left, inches_Front_Left;
    int duration_Back_Left,inches_Back_Left;
    int duration_Top_Left,inches_Top_Left;
    int duration_Top_Right,inches_Top_Right;
    int duration_Front_Right,inches_Front_Right;
    int duration_Back_Right,inches_Back_Right;
    int duration_Bottom_Right,inches_Bottom_Right;
    int duration_Bottom_Left,inches_Bottom_Left;
};
