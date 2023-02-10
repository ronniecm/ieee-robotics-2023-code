#include "Drivetrain.h"

#define FL_in1 14
#define FL_in2 15

#define FR_in1 36
#define FR_in2 37

#define BL_in1 5
#define BL_in2 4

#define BR_in1 24
#define BR_in2 6

Drivetrain::Drivetrain()
{
    enc[0] = new Encoder(22, 23);
    enc[1] = new Encoder(34, 33);
    enc[2] = new Encoder(1, 2);
    enc[3] = new Encoder(32, 31);

    for (int i = 0; i < 4; i++)
    {
        speedController[i] = new PID(&in[i], &out[i], &setpoint[i], kP[i], kI, kD[i], DIRECT);
    }
}

void Drivetrain::mecanumDrive(float x, float y, float z)
{
    double frontLeft = y + x + z;
    double frontRight = y - x - z;
    double backLeft = y - x + z;
    double backRight = y + x - z;

    double frontLeftRPM = frontLeft * 200.0 / 2.0;
    double frontRightRPM = frontRight * 200.0 / 2.0;
    double backLeftRPM = backLeft * 200.0 / 2.0;
    double backRightRPM = backRight * 200.0 / 2.0;
    
    frontLeft = frontLeft * 255;
    frontRight = frontRight * 255;
    backLeft = backLeft * 255;
    backRight = backRight * 255;

    setpoint[0] = abs(frontLeftRPM);
    setpoint[1] = abs(frontRightRPM);
    setpoint[2] = abs(backLeftRPM);
    setpoint[3] = abs(backRightRPM);

    for (int i = 0; i < 4; i++)
    {
        in[i] = abs(finalRpm[i]);
        speedController[i]->Compute();
    }
    
    // Apply the calculated values to the motor control pins
    if (frontLeft >= 0)
    {
        analogWrite(FL_in1, 0);
        analogWrite(FL_in2, out[0]);
    }
    else
    {
        analogWrite(FL_in1, out[0]);
        analogWrite(FL_in2, 0);
    }
    if (frontRight >= 0)
    {
        analogWrite(FR_in1, out[1]);
        analogWrite(FR_in2, 0);
    }
    else
    {
        analogWrite(FR_in1, 0);
        analogWrite(FR_in2, out[1]);
    }
    if (backLeft >= 0)
    {
        analogWrite(BL_in1, out[2]);
        analogWrite(BL_in2, 0);
    }
    else
    {
        analogWrite(BL_in1, 0);
        analogWrite(BL_in2, out[2]);
    }
    if (backRight >= 0)
    {
        analogWrite(BR_in1, out[3]);
        analogWrite(BR_in2, 0);
    }
    else
    {
        analogWrite(BR_in1, 0);
        analogWrite(BR_in2, out[3]);
    }
}

void Drivetrain::calcRPM()
{
    count++;
    for (int i = 0; i < 4; i++)
    {
        int currPos = enc[i]->read();
        int deltaPos = currPos - prevPos[i];
        double revs = deltaPos / 2678.0;
        double rpmCalc = revs * 60000;
        prevPos[i] = currPos;
        rpmSum[i] += rpmCalc;
        if (count >= 10)
        {
            finalRpm[i] = rpmSum[i] / 10.0;
            rpmSum[i] = 0;
        }
    }
    if (count >= 10)
    {
        count = 0;
    }
}

Encoder *Drivetrain::getEnc(int i)
{
    return enc[i];
}

PID *Drivetrain::getController(int i)
{
    return speedController[i];
}

double Drivetrain::getRPM(int i) {
    return finalRpm[i];
}