// // #include <Wire.h>
#include "Adafruit_TCS34725softi2c.h"
#include "dispensingdropoff.h"
#define TCS34725_IntegrationTime_614MS 0x00

// You can use any digital pin for emulate SDA / SCL
#define SDApin1 52
#define SCLpin1 9

#define SDApin2 50
#define SCLpin2 2

/* Initialise with specific int time and gain values and custom SDA / SCL pin */
Adafruit_TCS34725softi2c tcs1 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X, SDApin1, SCLpin1);
Adafruit_TCS34725softi2c tcs2 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X, SDApin2, SCLpin2);
// Motor A connections
int enA = 13;
int in1 = 22;
int in2 = 24;
// Motor B connections
int enB = 12;
int in3 = 28;
int in4 = 26;
// Motor C connections
int enC = 11;
int in5 = 30;
int in6 = 32;
// Motor D connections
int enD = 10;
int in7 = 34;
int in8 = 36;

bool done = false;


void Dispensing::setup_bot() {
  // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(enC, OUTPUT);
    pinMode(enD, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(in5, OUTPUT);
    pinMode(in6, OUTPUT);
    pinMode(in7, OUTPUT);
    pinMode(in8, OUTPUT);
    
    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(in5, LOW);
    digitalWrite(in6, LOW);
    digitalWrite(in7, LOW);
    digitalWrite(in8, LOW);
    
    Serial.begin(9600);
    
    if (tcs1.begin()) {
    Serial.println("Found sensor");
    } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
    }
    
    // Now we're ready to get readings!
}
void Dispensing::forward(int mspeed)
{
  analogWrite(enA, mspeed);
  analogWrite(enB, mspeed);
  analogWrite(enC, mspeed);
  analogWrite(enD, mspeed);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void Dispensing::backward(int mspeed)
{
  analogWrite(enA, mspeed);
  analogWrite(enB, mspeed);
  analogWrite(enC, mspeed);
  analogWrite(enD, mspeed);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void Dispensing::left(int mspeed)
{
  analogWrite(enA, mspeed);
  analogWrite(enB, mspeed);
  analogWrite(enC, mspeed);
  analogWrite(enD, mspeed);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void Dispensing::right(int mspeed)
{
  analogWrite(enA, mspeed);
  analogWrite(enB, mspeed);
  analogWrite(enC, mspeed);
  analogWrite(enD, mspeed);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void Dispensing::halt()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
}

bool Dispensing::checkcolor()
{
  uint16_t r1, g1, b1, c1, r2, g2, b2, c2;
  int color1, color2;
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  // check if blue
  if ((r1 < 95) & (b1 > r1) & (b1 > g1) & (g1 < 95))
  {
    Serial.print("blue");
    Serial.println();
    return false;
  }
  // check if green
  else if ((r1 < 95) & (b1 < 95) & (g1 > 95))
  {
    Serial.print("green");
    Serial.println();
    return false;
  }
  // check if red
  else if ((r1 > 95) & (b1 < 95) & (g1 < 95))
  {
    Serial.print("red");
    Serial.println();
    return false;
  }
  // check if white
  else if ((r1 > 5) & (b1 > 5) & (g1 > 5))
  {
    if ((r2 > 5) & (b2 > 5) & (g2 > 5))
    {
      Serial.print("white");
      Serial.println();
      return true;
    }
    else
    {
      // do nothing
      return false;
    }
  }
  // check if black
  else if ((r1 < 5) & (b1 < 5) & (g1 < 5))
  {
    Serial.print("black");
    Serial.println();
    return false;
  }
  else
  {
    Serial.print("N/A");
    Serial.println();
    return false;
  }
}