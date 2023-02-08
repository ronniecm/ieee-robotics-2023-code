/*
IR Sensor: SHARP 0A51SK
Name GP2Y0A51SK0F
This reads the sensor from analog pin A0 and from this analog value this maps to a voltage
From the voltage the next step is to map the value to a distance which was done through a inverse plot
of the output coltage to distacne from the Datasheet
Contributers: Araceli, Azam, Jay
*/
#include <math.h>
void setup() 
{
  Serial.begin(9600);
}

void loop()
{
  int reading = analogRead(A0);
  float voltage = reading * (5.0 / 1023.0);//find output voltage
  //Formula below was from the linear approximation of the inverse of the ouptut voltage to distance from the datasheet plot
  float distance = (3.2011*pow(voltage,4)) - (21.485*pow(voltage,3)) + (53.838*pow(voltage,2))-(62.73*voltage)+(29.7);
  //distance in cm
  float average = 0.0;
  for(int i =0;i<10;i++)
  {
    average+=distance;
  }
  average = average/10;
  delay(100);
  Serial.println(average);   // print the distance
  average = 0.0;
  
}