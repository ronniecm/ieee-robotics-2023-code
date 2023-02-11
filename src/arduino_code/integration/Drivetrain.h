#include <PID_v1.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <Arduino.h>
#include <vector>

/*
 * Author: Ronnie Mohapatra 
 * Description: Outlines header file for Drivetrain object
 */
class Drivetrain {
public:
    Drivetrain(); //constructor
    ~Drivetrain(); //destructor
    Encoder* getEnc(int i); //get method for an encoder object
    PID* getController(int i); //get method for a PID speed controller
    double getRPM(int i); //get method for retrieving RPM of a wheel
    void mecanumDrive(float x, float y, float z); //mecanum drive function
    void calcRPM(); //calculates RPM of all motors
private:
    Encoder* enc[4]; //encoder array
    int prevPos[4]; //used in RPM calculations
    int count; //used in RPM calculations

    double rpmSum[4]; //used in RPM calculations
    double finalRpm[4]; //official RPM array for all motors

    //PID values for all 4 speed controllers
    double kP[4] = {30,30,30,30};
    double kI = 0.0;
    double kD[4] = {0,0,0,0};

    //Input, output, and setpoint arrays for all PID speed controllers
    double in[4];
    double out[4];
    double setpoint[4];

    PID* speedController[4]; //array of PID speed controllers for all motors
};
