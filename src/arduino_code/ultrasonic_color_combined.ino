//Front Left trig and echo pins and distance
#define Front_Left_trigPin 11    // Trigger
#define Front_Left_echoPin 6  // Echo
long duration_FrontLeft, cm_FrontLeft, inches_Front_Left;

//Back Left trig and echo pins and distance
#define Back_Left_trigPin 10
#define Back_Left_echoPin 9
long duration_Back_Left, cm_Back_Left, inches_Back_Left;

//Top Left trig and echo pins and distance
#define Top_Left_trigPin 13
#define Top_Left_echoPin 8
long duration_Top_Left,cm_Top_Left,inches_Top_Left;
int color = 0; //red = 1, green = 2

//COLOR SENSOR

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
#define RED_LED_PIN 4
#define GREEN_LED_PIN 7
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(Front_Left_trigPin, OUTPUT);
  pinMode(Front_Left_echoPin, INPUT);

  pinMode(Back_Left_trigPin, OUTPUT);
  pinMode(Back_Left_echoPin, INPUT);
  
  pinMode(Top_Left_trigPin,OUTPUT);
  pinMode(Top_Left_echoPin,INPUT);

  //COLOR SENSOR

  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // use these three pins to drive an LED
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
  
}
 
void loop() {
  //Below is front left calculations
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(Front_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Left_trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(Front_Left_echoPin, INPUT);
  duration_FrontLeft = pulseIn(Front_Left_echoPin, HIGH);
  // Convert the time into a distance
  cm_FrontLeft = (duration_FrontLeft/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches_Front_Left = (duration_FrontLeft/2) / 74;   // Divide by 74 or multiply by 0.0135
  //Serial.print(inches_Front_Left);
  //Serial.println(" Inches Front Left");
  //delay(1000);


  //below is back left calculations
  digitalWrite(Back_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Left_trigPin, LOW);
  pinMode(Back_Left_echoPin, INPUT);
  duration_Back_Left = pulseIn(Back_Left_echoPin, HIGH);
  // Convert the time into a distance
  cm_Back_Left = (duration_Back_Left/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches_Back_Left = (duration_Back_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
  //Serial.print(inches_Back_Left);
  //Serial.println(" Back Left ");
  //delay(1000);


  //below is Top Left calculations
  digitalWrite(Top_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Left_trigPin,LOW);
  pinMode(Top_Left_echoPin, INPUT);
  duration_Top_Left = pulseIn(Top_Left_echoPin, HIGH);
  cm_Top_Left = (duration_Top_Left/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches_Top_Left = (duration_Top_Left/2) / 74;  
  //Serial.print(inches_Top_Left);
  //Serial.println(" Top Left ");
  //delay(500);

  float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

//  Serial.print("\t");
//  Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  Serial.print("\n");

   if((int(red)>int(green)) && (int(red)>int(blue)) && (int(red)>160))
  {
    Serial.println("RED");
    digitalWrite(RED_LED_PIN, HIGH);
    delay(100);
    color = 1;
    
  }
  else if((int(green)>int(red)) && (int(green)>int(blue)) && (int(green)>95))
  {
    //Serial.println("GREEN");
    //digitalWrite(GREEN_LED_PIN, HIGH);
    //delay(100);
    color = 2;
  }
  else
  {
    //digitalWrite(RED_LED_PIN, LOW);
    //digitalWrite(GREEN_LED_PIN, LOW);
  }

  if(inches_Top_Left <= 3)
  {
    if(inches_Front_Left <= 3)
    {
      Serial.print("CORNER Detected");
      Serial.println();

      if(color == 1)
      {
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.print("ALERT");
      }
      if(color == 2)
      {
        digitalWrite(GREEN_LED_PIN, HIGH);
      } 
    } 
  }
  else if((inches_Front_Left <= 3) && (inches_Back_Left <=3))
  {
    //if((inches_Back_Left-inches_Front_Left <= 0.5 && inches_Back_Left-inches_Front_Left >= 0) || (inches_Front_Left-inches_Back_Left <= 0.5 && inches_Front_Left-inches_Back_Left >= 0))
    if(inches_Back_Left == inches_Front_Left)
      {
        Serial.print("LEFT SIDE - ALIGNED");
        Serial.println();
      }
      else
      {
        Serial.print("LEFT SIDE - UNALIGNED");
        Serial.println();
      }
  }
  else
  {
    Serial.print("On Gameboard");
    Serial.println();
    
  }
 

  delay(250);



}
