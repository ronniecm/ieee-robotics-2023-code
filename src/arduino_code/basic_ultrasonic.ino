/*
Code was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be six ultrasonic sensors around the robot (2 on the left side, 2 on the right side, one on front, one on back)
and create distance calculations of knowing where the robot is relatie to the gameboard
*/

//Front Left trig and echo pins and distance
#define Front_Left_trigPin 11    // Trigger
#define Front_Left_echoPin 12    // Echo
long duration_FrontLeft, cm_FrontLeft, inches_FrontLeft;

//Back Left trig and echo pins and distance
#define Back_Left_trigPin 10
#define Back_Left_echoPin 9
long duration_Back_Left, cm_Back_Left, inches_Back_Left;

//Top Left trig and echo pins and distance
#define Top_Left_trigPin 13
#define Top_Left_echoPin 8
long duration_Top_Left,cm_Top_Left,inches_Top_Left;

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
  inches_FrontLeft = (duration_FrontLeft/2) / 74;   // Divide by 74 or multiply by 0.0135
  Serial.print(inches_FrontLeft);
  Serial.println(" Inches at Front Left Sensor");
  delay(1000);


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
  Serial.print(inches_Back_Left);
  Serial.println(" Inches at Back Left Sensor");
  delay(1000);


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
  Serial.print(inches_Top_Left);
  Serial.println(" Inches at Top Left Sensor");
  delay(1000);



}
