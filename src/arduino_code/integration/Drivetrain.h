#include <PID_v1.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <Arduino.h>
#include "MovingAverageFilter.hpp"
#include <vector>


/*
 * Author: Ronnie Mohapatra 
 * Description: Outlines header file for Drivetrain object
 */
class Drivetrain {
public:
    Drivetrain(); //constructor
    ~Drivetrain(); //destructor
    int getEnc(int i); //get method for an encoder object
    PID* getController(int i); //get method for a PID speed controller
    double getRPM(int i); //get method for retrieving RPM of a wheel
    void mecanumDrive(float x, float y, float z); //mecanum drive function
    void calcRPM(); //calculates RPM of all motors
    // void calculateSpeed(int i);
private:
    Encoder* enc[4]; //encoder array
    int prevPos[4]; //used in RPM calculations
    int count; //used in RPM calculations

    double rpmSum[4]; //used in RPM calculations
    double finalRpm[4]; //official RPM array for all motors

    //PID values for all 4 speed controllers
    double kPhigh[4] = {14.5, 14.5, 14.5, 14.5};
    double kIhigh[4] = {20, 20, 20, 20};
    double kDhigh[4] = {0.17, 0.17, 0.17, 0.17};

    double kPlow[4] = {5, 5, 5, 5};
    double kIlow[4] = {15, 15, 15, 15};
    double kDlow[4] = {0.15, 0.15, 0.15, 0.15};

    //Input, output, and setpoint arrays for all PID speed controllers
    double in[4];
    double out[4];
    double setpoint[4];

    PID* speedController[4]; //array of PID speed controllers for all motors

    unsigned long previousTime;
    double integralError;
    double lastError;

    // Array of moving average filters for each motor
    MovingAverageFilter* rpmFilter[4];

};
