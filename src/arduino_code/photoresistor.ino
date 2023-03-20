#include "ros.h"
#include "std_msgs/Int8.h"
//Link for wiring: https://www.instructables.com/How-to-use-a-photoresistor-or-photocell-Arduino-Tu/
//(only need photoresistor, 10k resistor, A0 pin, 5V, ground)
ros::NodeHandle nh; //creating ros node
std_msgs::Int8 redLEDmsg; 
int pResistor = A0; // Photoresistor at Arduino analog pin A0
ros::Publisher RedLED("/bot/redled", &redLEDmsg);
//Note: The Red LED must be directly above the photoresistor!
/*
 * This will publish an Int8 type with a value of 1 or zero
 * If the red led is detected it will publish a 1
 * Else it will print a 0
 */
void setup()
{
  //Serial.begin(9600);
  nh.initNode();
  pinMode(pResistor, INPUT);
  nh.advertise(RedLED);
}

void loop()
{
  int pValue = analogRead(pResistor);
  //Serial.println(pValue);
  //Analog values for red color range below 
  /* Testing 
    <1cm: 960
    1cm:  938
    2cm:  846
    3cm:  790
    4cm:  735
    5cm:  727
  */
  //750 to 820
  int lower_range = 720;
  int upper_range = 960;
  if (pValue >lower_range && pValue <upper_range)
  {
    //Serial.println("START");
    redLEDmsg.data = 1;
  }
  else {
    redLEDmsg.data = 0;
  }
  RedLED.publish(&redLEDmsg);
  nh.spinOnce();
  delay(60);
}
