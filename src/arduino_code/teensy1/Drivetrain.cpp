#include "Drivetrain.h"

// Measured encoder ticks per single revolution

Drivetrain::Drivetrain()
{
    //First initialize all 4 encoder objects
    enc[0] = new Encoder(FL_enc1, FL_enc2);
    enc[1] = new Encoder(FR_enc1, FR_enc2);
    enc[2] = new Encoder(BL_enc1, BL_enc2);
    enc[3] = new Encoder(BR_enc1, BR_enc2);
  
    //Next initialize all 4 PID speed controllers with proper PID and setpoint values
    for (int i = 0; i < 4; i++)
    {
        speedController[i] = new PID(&in[i], &out[i], &setpoint[i], this->kP[i], this->kI[i], this->kD[i], DIRECT);
        speedController[i]->SetMode(AUTOMATIC);
        speedController[i]->SetSampleTime(10);
    }

    // Initialize all 4 moving average filters
    for (int i = 0; i < 4; i++)
    {
        rpmFilter[i] = new MovingAverageFilter(200);
    }
  
    //initialize count to 0
    count = 0;
}

Drivetrain::~Drivetrain()
{
    // Destructor deletes any pointers that were used
    delete[] enc;
    delete[] speedController;
    delete[] rpmFilter;
}

void Drivetrain::mecanumDrive(float x, float y, float z)
{
    // calculate power for each wheel with respective formula
    double frontLeft = y + x + z;
    double frontRight = y - x - z;
    double backLeft = y - x + z;
    double backRight = y + x - z;

    double frontLeftRPM = abs(frontLeft) * 100;
    double frontRightRPM = abs(frontRight) * 100;
    double backLeftRPM = abs(backLeft) * 100;
    double backRightRPM = abs(backRight) * 100;
    
    setpoint[0] = frontLeftRPM;
    setpoint[1] = frontRightRPM;
    setpoint[2] = backLeftRPM;
    setpoint[3] = backRightRPM;

    for(int i = 0; i < 4; i++) {
      in[i] = abs(finalRpm[i]);  
      if (setpoint[i] == 0) {
        out[i] = 0;  
      }  
      else {
        speedController[i]->Compute();  
      }
    }
    
    // Front left motor
    if (frontLeft >= 0)
    {
        // Clockwise rotation
        analogWrite(FL_in1, 0);
        analogWrite(FL_in2, out[0]);
    }
    else
    {
        analogWrite(FL_in1,  out[0]);
        analogWrite(FL_in2, 0);
    }   

    if (frontRight >= 0)
    {
        // Clockwise rotation
        analogWrite(FR_in1, 0);
        analogWrite(FR_in2, out[1]);
    }
    else
    {
        analogWrite(FR_in1,  out[1]);
        analogWrite(FR_in2, 0);
    } 
    
    if (backLeft >= 0)
    {
        // Clockwise rotation
        analogWrite(BL_in1, 0);
        analogWrite(BL_in2, out[2]);
    }
    else
    {
        analogWrite(BL_in1,  out[2]);
        analogWrite(BL_in2, 0);
    }   

    if (backRight >= 0)
    {
        // Clockwise rotation
        analogWrite(BR_in1, 0);
        analogWrite(BR_in2, out[3]);
    }
    else
    {
        analogWrite(BR_in1,  out[3]);
        analogWrite(BR_in2, 0);
    }   
}

void Drivetrain::calcRPM()
{
    // Get the current time in microseconds
    unsigned long currentTime = micros();
    
    // Calculate the elapsed time since the last call (in milliseconds)
    double elapsedTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    for (int i = 0; i < 4; i++)
    {
        int currPos = enc[i]->read();
        double deltaPos = currPos - prevPos[i];
        double revs = deltaPos / TICKS_PER_REV;

        // Calculate the RPM based on the elapsed time
        double rpmCalc = revs * (60000.0 / elapsedTime);
        prevPos[i] = currPos;
        
        // // accumulate the RPM values
        // rpmSum[i] += rpmCalc;

        // Apply moving average filter
        finalRpm[i] = rpmFilter[i]->process(rpmCalc);
    }
  
}


// Returns the encoder object at the given index
int Drivetrain::getEnc(int i)
{
    return enc[i]->read();
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

void Drivetrain::tunePID(double kPnew, double kInew, double kDnew) {
    
    for (int i = 0; i < 4; i++)
    { 
      this->getController(i)->SetTunings(kPnew, kInew, kDnew);
    }
}

void Drivetrain::printEncoders() {
  for(int i = 0; i < 4; i++) {
    Serial.print(getEnc(i));
    Serial.print(" ");
  }
  Serial.println();
}

void Drivetrain::printRPM() {
  for(int i = 0; i < 4; i++) {
    Serial.print(getRPM(i));
    Serial.print(" ");
  }
  Serial.println();
}
