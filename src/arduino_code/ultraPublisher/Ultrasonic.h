#include "Arduino.h"

class Ultrasonic
{
public:
    float getFrontLeftDistance();
    float getBackLeftDistance();
    float getTopLeftDistance();
    float getTopRightDistance();
    float getFrontRightDistance();
    float getBackRightDistance();
    float getBottomRightDistance();
    float getBottomLeftDistance();

private:
    float duration_Front_Left, inches_Front_Left;
    float duration_Back_Left,inches_Back_Left;
    float duration_Top_Left,inches_Top_Left;
    float duration_Top_Right,inches_Top_Right;
    float duration_Front_Right,inches_Front_Right;
    float duration_Back_Right,inches_Back_Right;
    float duration_Bottom_Right,inches_Bottom_Right;
    float duration_Bottom_Left,inches_Bottom_Left;
};
