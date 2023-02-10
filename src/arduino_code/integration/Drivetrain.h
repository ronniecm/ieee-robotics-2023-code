#include <PID_v1.h>
#include <TimerOne.h>
#include <Encoder.h>
#include "Arduino.h"

class Drivetrain {
public:
    Drivetrain();
    Encoder* getEnc(int i);
    PID* getController(int i);
    double getRPM(int i);
    void mecanumDrive(float x, float y, float z);
    void calcRPM();
private:
    Encoder* enc[4];
    int prevPos[4];
    int count;

    double rpmSum[4];
    double finalRpm[4];

    double kP[4] = {8,8,8,8};
    double kI = 0.0;
    double kD[4] = {0,0,0,0};
    double in[4];
    double out[4];
    double setpoint[4];

    PID* speedController[4];
};
