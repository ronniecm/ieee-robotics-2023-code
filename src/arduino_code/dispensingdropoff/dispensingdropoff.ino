#include <Wire.h>
#include "dispensingdropoff.h"
#include "ros.h"
#include "std_msgs/Int8.h"

ros::NodeHandle nh;

Dispensing dispense;

std_msgs::Int8 colorMsg;

ros::Publisher color("/bot/color", &colorMsg);
void setup(void)
{
  dispense.setup_bot();
  colorMsg.data = 0;
  nh.advertise(color);
}
void loop(void) {
  
  if (dispense.checkcolor()) {
    colorMsg.data = 1;
    GripperRotate.publish(&colorMsg);
  }
  else {
    colorMsg.data = 0;
    GripperRotate.publish(&colorMsg);
  }
}
