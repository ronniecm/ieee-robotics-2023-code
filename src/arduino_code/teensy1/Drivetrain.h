#include <PID_v1.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <Arduino.h>
#include "MovingAverageFilter.hpp"
#include <vector>

#define FL_in1 2
#define FL_in2 1
#define FL_enc1 3
#define FL_enc2 4

#define FR_in1 7
#define FR_in2 8
#define FR_enc1 25
#define FR_enc2 24

#define BR_in1 5
#define BR_in2 6
#define BR_enc1 12
#define BR_enc2 11

#define BL_in1 28
#define BL_in2 29
#define BL_enc1 31
#define BL_enc2 32

#define TICKS_PER_REV 2678
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
    void tunePID(double kPnew, double kInew, double kDnew); //tunes PID values for all motors
    void printEncoders();
    void printRPM();
    
private:
    Encoder* enc[4]; //encoder array
    int prevPos[4]; //used in RPM calculations
    int count; //used in RPM calculations

    double rpmSum[4]; //used in RPM calculations
    double finalRpm[4]; //official RPM array for all motors

    //PID values for all 4 speed controllers
    double kP[4] = {5, 5, 5, 5};
    double kI[4] = {15, 15, 15, 15};
    double kD[4] = {0.15, 0.15, 0.15, 0.15};

    //Input, output, and setpoint arrays for all PID speed controllers
    double in[4];
    double out[4];
    double setpoint[4];

    PID* speedController[4]; //array of PID speed controllers for all motors

    unsigned long previousTime;

    // Array of moving average filters for each motor
    MovingAverageFilter* rpmFilter[4];
};
