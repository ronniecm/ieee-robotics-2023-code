#include "Drivetrain.h"

// Front left motor pin definition
#define FL_in1 22 
#define FL_in2 23
#define FL_enc_in1 20
#define FL_enc_in2 21

// Front right motor pin definition
#define FR_in1 36
#define FR_in2 37
#define FR_enc_in1 34
#define FR_enc_in2 35

// Back left motor pin definition
#define BL_in1 2
#define BL_in2 3
#define BL_enc_in1 4
#define BL_enc_in2 5

// Back right motor pin definition
#define BR_in1 28
#define BR_in2 29
#define BR_enc_in1 30
#define BR_enc_in2 31

// Measured encoder ticks per single revolution
#define TICKS_PER_REV 2678

Drivetrain::Drivetrain()
{
    //First initialize all 4 encoder objects
    enc[0] = new Encoder(FL_enc_in1, FL_enc_in2);
    enc[1] = new Encoder(FR_enc_in1, FR_enc_in2);
    enc[2] = new Encoder(BL_enc_in1, BL_enc_in2);
    enc[3] = new Encoder(BR_enc_in1, BR_enc_in2);

    //Next initialize all 4 PID speed controllers with proper PID and setpoint values
    for (int i = 0; i < 4; i++)
    {
        speedController[i] = new PID(&in[i], &out[i], &setpoint[i], kP[i], kI, kD[i], DIRECT);
        speedController[i]->SetMode(AUTOMATIC);
        speedController[i]->SetSampleTime(10);
    }

    //initialize count to 0
    count = 0;
}

Drivetrain::~Drivetrain()
{
    // Destructor deletes any pointers that were used
    delete[] enc;
    delete[] speedController;
}

void Drivetrain::mecanumDrive(float x, float y, float z)
{
    // calculate power for each wheel with respective formula
    double frontLeft = y + x + z;
    double frontRight = y - x - z;
    double backLeft = y - x + z;
    double backRight = y + x - z;

    // map -1.0 to 1.0 range to -100 to 100 RPM
    double frontLeftRPM = frontLeft * 100.0;
    double frontRightRPM = frontRight * 100.0;
    double backLeftRPM = backLeft * 100.0;
    double backRightRPM = backRight * 100.0;

    /*
    frontLeft = frontLeft * 255;
    frontRight = frontRight * 255;
    backLeft = backLeft * 255;
    backRight = backRight * 255;
    */

    // set the setpoint of each PID speed controller
    setpoint[0] = abs(frontLeftRPM);
    setpoint[1] = abs(frontRightRPM);
    setpoint[2] = abs(backLeftRPM);
    setpoint[3] = abs(backRightRPM);

    // set the input of each PID speed controller and compute the output
    for (int i = 0; i < 4; i++)
    {
        in[i] = abs(finalRpm[i]);
        speedController[i]->Compute();
    }

    // Apply the calculated values to the motor control pins
    
    // Front left motor
    if (frontLeft >= 0)
    {
        // Clockwise rotation
        analogWrite(FL_in1, 0);
        analogWrite(FL_in2, out[0]);
    }
    else
    {
        // Counter-clockwise rotation
        analogWrite(FL_in1, out[0]);
        analogWrite(FL_in2, 0);
    }

    // Front right motor
    if (frontRight >= 0)
    {
        // Clockwise rotation
        analogWrite(FR_in1, out[1]);
        analogWrite(FR_in2, 0);
    }
    else
    {
        // Counter-clockwise rotation
        analogWrite(FR_in1, 0);
        analogWrite(FR_in2, out[1]);
    }

    // Back left motor
    if (backLeft >= 0)
    {
        // Clockwise rotation
        analogWrite(BL_in1, out[2]);
        analogWrite(BL_in2, 0);
    }
    else
    {
        // Counter-clockwise rotation
        analogWrite(BL_in1, 0);
        analogWrite(BL_in2, out[2]);
    }

    // Back right motor
    if (backRight >= 0)
    {
        // Clockwise rotation
        analogWrite(BR_in1, out[3]);
        analogWrite(BR_in2, 0);
    }
    else
    {
        // Counter-clockwise rotation
        analogWrite(BR_in1, 0);
        analogWrite(BR_in2, out[3]);
    }
}

void Drivetrain::calcRPM()
{
    //this functions gets called every millisecond
    //everytime the functions gets called increment the call count variable 'count'
    count++;
    for (int i = 0; i < 4; i++)
    {
        //for each motor, find its current encoder position and then how much it has changed from its last reading
        //then divide that delta value by TICK_PER_REV to get the number of revolutions performed in that millisecond
        //finally to retrieve the RPM value multiply that value by 60,000 and set the prevPos[i] to the currPos saved earlier
        //every 10 calls to this functions, the samples are averaged and saved into finalRpm array
        int currPos = enc[i]->read();
        double deltaPos = currPos - prevPos[i];
        double revs = deltaPos / TICKS_PER_REV;
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

// Returns the encoder object at the given index
Encoder *Drivetrain::getEnc(int i)
{
    return enc[i];
}

// Returns the PID object at the given index
PID *Drivetrain::getController(int i)
{
    return speedController[i];
}

// Returns the final RPM value at the given index
double Drivetrain::getRPM(int i) {
    return finalRpm[i];
}
