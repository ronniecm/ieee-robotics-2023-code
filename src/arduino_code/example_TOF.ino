/*
View code for using single VL6180X sensors: https://robojax.com/course1/lecture76
 * Original code and library by https://github.com/adafruit/Adafruit_VL6180X
 * Written/updated by Ahmad Shamshiri for Robojax Robojax.com
 * on Mar 12, 2021  in Ajax, Ontario, Canada  
 * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>. 
*/
/*
Left Sensor Pins:
SDA: A4
SCL: A5
SHDN: Digital Pin 7
Power
Ground
*/

/*
Right Sensor Pins:
SDA
SCL
SHDN: Digital Pin 6
Power
Ground
*/
#include <Adafruit_VL6180X.h>

// address we will assign if dual sensor is present
#define LEFT_SENSOR_ADDRESS 0x30
#define RIGHT_SENSOR_ADDRESS 0x31

// Additional Digital Pin for each sensor
#define SHT_LEFT_SENSOR 7
#define SHT_RIGHT_SENSOR 6


// objects for the VL6180X
Adafruit_VL6180X LEFT_SENSOR = Adafruit_VL6180X();
Adafruit_VL6180X RIGHT_SENSOR = Adafruit_VL6180X();


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
*/
void setID() {
  // all reset
  digitalWrite(SHT_LEFT_SENSOR, LOW);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);

  delay(10);

  // all unreset
  digitalWrite(SHT_LEFT_SENSOR, HIGH);
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);

  delay(10);

  // activating left sensor and reseting right sensor
  digitalWrite(SHT_LEFT_SENSOR, HIGH);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);


  // initing Left sensor
  if (!LEFT_SENSOR.begin()) {
    Serial.println(F("Failed to boot LEFT_SENSOR VL6180X"));
    while (1);
  }
  LEFT_SENSOR.setAddress(LEFT_SENSOR_ADDRESS);
  delay(10);

  // activating RIGHT_SENSOR
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);
  delay(10);

  //initing RIGHT_SENSOR
  if (!RIGHT_SENSOR.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  RIGHT_SENSOR.setAddress(RIGHT_SENSOR_ADDRESS);
  delay(10);

 
}

float readSensorValue(Adafruit_VL6180X &vl) {

  //float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  float range = vl.readRange();
  float to_cm = range/10;//convert mm to cm
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) 
  {
      return to_cm;//able to receive reading on the TOF sensor, return distance
  }
  else
  {
  // uint8_t readRange(void); will never return a negative value 
    return -1.0;//else, an error occured with reading, flag is negative number
  }


}


void check_measurement(float left_sensor, float right_sensor)
{
  if(left_sensor > right_sensor)
  {
    float difference = left_sensor - right_sensor;
    if(difference > 2)//sensor left is greater then sensor right by 2 cm
    {
      Serial.println(" Bot is favored to the leftside and not alligned by ");
      Serial.print(difference);
    }
    else
    {
      Serial.println(" Bot is favored to the leftside but within threshold ");
    }
  
  }
  else if(right_sensor > left_sensor)
  {
    float difference = right_sensor - left_sensor;
    if(difference > 2)//sensor right is greater then sensor left by 2 cm
    {
      
      Serial.println(" Bot is favored to the rightside and not alligned by ");
      Serial.print(difference);
    }
    else
    {
      Serial.println(" Bot is favored to the rightside but within threshold ");
    }
  }
  else
  {
   Serial.println("Bot is alligned"); 
  }
  Serial.println();



}



void read_sensors() 
{
  float sensor_left = readSensorValue(LEFT_SENSOR);
  float sensor_right = readSensorValue(RIGHT_SENSOR);
  //check to make sure each sensor reading is valid then continue with the calculations
  if((sensor_left != (-1.0)) && (sensor_right != (-1.0)))
  {
     check_measurement(sensor_left,sensor_right);
     
     /*
     Serial.print(sensor_left);
     Serial.println(" LEFT SENSOR ");

     Serial.print(sensor_right);
     Serial.println(" RIGHT SENSOR ");
     */
     delay(2000);
     
  }
  else
  {
    Serial.println("One sensor returned invalid value");
  }

}





//===============================================================
// Setup
//===============================================================
void setup() 
{
  Serial.begin(115200);

  pinMode(SHT_LEFT_SENSOR, OUTPUT);
  pinMode(SHT_RIGHT_SENSOR, OUTPUT);

  digitalWrite(SHT_LEFT_SENSOR, LOW);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);

  setID();


}

//===============================================================
// Loop
//===============================================================
void loop()
{
  read_sensors();
  delay(100);
}
