#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

const int buttonPin = 13;

int pos = 0;    // variable to store the servo position
int buttonState = 0;

uint16_t r, g, b, c, colorTemp, lux;

void setup() {
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  // Now we're ready to get readings!
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  
  
  if (buttonState == HIGH) { // button is pressed
    tcs.getRawData(&r, &g, &b, &c);
    
    if (r > g && r > b) { // red 
      myservo.write(0);
    }
    else if (g > r && g > b) { // green
      myservo.write(180);
    }
    else {
      myservo.write(90);
    }
  }
  else {
    myservo.write(90);
  }
}
