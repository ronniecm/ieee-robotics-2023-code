/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be six ultrasonic sensors around the robot (2 on the left side, 2 on the right side, one on front, one on back)
and create distance calculations of knowing where the robot is relative to the gameboard using a coordinate system
*/

//Front Left trig and echo pins and distance variables
#define Front_Left_trigPin 4    // Trigger
#define Front_Left_echoPin 5    // Echo
int duration_Front_Left,inches_Front_Left;

//Back Left trig and echo pins and distance variables
#define Back_Left_trigPin 3
#define Back_Left_echoPin 2
int duration_Back_Left,inches_Back_Left;

//Top Left trig and echo pins and distance variables
#define Top_Left_trigPin 6
#define Top_Left_echoPin 7
int duration_Top_Left,inches_Top_Left;

//Top Right trig and echo pins and distance variables
#define Top_Right_trigPin 22
#define Top_Right_echoPin 23
int duration_Top_Right,inches_Top_Right;

//Front Right trig and echo pins and distance variables
#define Front_Right_trigPin 9
#define Front_Right_echoPin 8
int duration_Front_Right,inches_Front_Right;

//Back Right trig and echo pins and distance variables
#define Back_Right_trigPin 11
#define Back_Right_echoPin 10
int duration_Back_Right,inches_Back_Right;
//Bottom Right trig and echo pins and distance variables

#define Bottom_Right_trigPin 13
#define Bottom_Right_echoPin 12
int duration_Bottom_Right,inches_Bottom_Right;

#define Bottom_Left_trigPin 24
#define Bottom_Left_echoPin 25
int duration_Bottom_Left,inches_Bottom_Left;

void setup() {
  //Serial Port begin
  Serial.begin(9600);
  //Define inputs and outputs (Trig and echo pins) of each of the ultrasonic sensors
  //Front Left
  pinMode(Front_Left_trigPin,OUTPUT);
  pinMode(Front_Left_echoPin,INPUT);
  //Back Left
  pinMode(Back_Left_trigPin,OUTPUT);
  pinMode(Back_Left_echoPin,INPUT);

  //Top Left
  pinMode(Top_Left_trigPin,OUTPUT);
  pinMode(Top_Left_echoPin,INPUT);

  //Top Right
  pinMode(Top_Right_trigPin,OUTPUT);
  pinMode(Top_Right_echoPin,INPUT);

  //Front Right
  pinMode(Front_Right_trigPin,OUTPUT);
  pinMode(Front_Right_echoPin,INPUT);

  //Back Right
  pinMode(Back_Right_trigPin,OUTPUT);
  pinMode(Back_Right_echoPin,INPUT);

  //Bottom Right
  pinMode(Bottom_Right_trigPin,OUTPUT);
  pinMode(Bottom_Right_echoPin,INPUT);

  //Bottom Left
  pinMode(Bottom_Left_trigPin,OUTPUT);
  pinMode(Bottom_Left_echoPin,INPUT);
}
 
void loop() 
{
  //for each ultrasonic sensor calculate the distance measurement in inches
  findFrontLeftDistance();
  findBackLeftDistance();
  findTopLeftDistance();
  findTopRightDistance();
  findFrontRightDistance();
  findBackRightDistance();
  findBottomRightDistance();
  findBottomLeftDistance();
  
}
//for each of the sensor measurements followed reference listed in comment header
void findFrontLeftDistance()
{
  digitalWrite(Front_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Left_trigPin, LOW);
  pinMode(Front_Left_echoPin, INPUT);
  duration_Front_Left = pulseIn(Front_Left_echoPin, HIGH);
  // Convert the time into a distance
  inches_Front_Left = (duration_Front_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
  Serial.print(inches_Front_Left);
  Serial.println(" Inches at Front Left Sensor");
  delay(1000);

}
void findBackLeftDistance()
{
//below is back left calculations
  digitalWrite(Back_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Left_trigPin, LOW);
  pinMode(Back_Left_echoPin, INPUT);
  duration_Back_Left = pulseIn(Back_Left_echoPin, HIGH);
  inches_Back_Left = (duration_Back_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
  Serial.print(inches_Back_Left);
  Serial.println(" Inches at Back Left Sensor");
  delay(1000);
}
void findTopLeftDistance()
{
 //below is Top Left calculations
  digitalWrite(Top_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Left_trigPin,LOW);
  pinMode(Top_Left_echoPin, INPUT);
  duration_Top_Left = pulseIn(Top_Left_echoPin, HIGH);
  inches_Top_Left = (duration_Top_Left/2) / 74;  
  Serial.print(inches_Top_Left);
  Serial.println(" Inches at Top Left Sensor");
  delay(1000);
}
void findTopRightDistance()
{
 //below is Top Left calculations
  digitalWrite(Top_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Right_trigPin,LOW);
  pinMode(Top_Right_echoPin, INPUT);
  duration_Top_Right = pulseIn(Top_Right_echoPin, HIGH);
  inches_Top_Right = (duration_Top_Right/2) / 74;  
  Serial.print(inches_Top_Right);
  Serial.println(" Inches at Top Right Sensor");
  delay(1000);
}
void findFrontRightDistance()
{
  digitalWrite(Front_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Right_trigPin,LOW);
  pinMode(Front_Right_echoPin, INPUT);
  duration_Front_Right = pulseIn(Front_Right_echoPin, HIGH);
  inches_Front_Right = (duration_Front_Right/2) / 74;  
  Serial.print(inches_Front_Right);
  Serial.println(" Inches at Front Right Sensor");
  delay(1000);
}
void findBackRightDistance()
{
  digitalWrite(Back_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Right_trigPin,LOW);
  pinMode(Back_Right_echoPin, INPUT);
  duration_Back_Right = pulseIn(Back_Right_echoPin, HIGH);
  inches_Back_Right = (duration_Back_Right/2) / 74;  
  Serial.print(inches_Back_Right);
  Serial.println(" Inches at Back Right Sensor");
  delay(1000); 
}

void findBottomRightDistance()
{
  digitalWrite(Bottom_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Bottom_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Bottom_Right_trigPin,LOW);
  pinMode(Bottom_Right_echoPin, INPUT);
  duration_Bottom_Right = pulseIn(Bottom_Right_echoPin, HIGH);
  inches_Bottom_Right = (duration_Bottom_Right/2) / 74;  
  Serial.print(inches_Bottom_Right);
  Serial.println(" Inches at Bottom Right Sensor");
  delay(1000); 

}
void findBottomLeftDistance()
{
  digitalWrite(Bottom_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Bottom_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Bottom_Left_trigPin,LOW);
  pinMode(Bottom_Left_echoPin, INPUT);
  duration_Bottom_Left = pulseIn(Bottom_Left_echoPin, HIGH);
  inches_Bottom_Left = (duration_Bottom_Left/2) / 74;  
  Serial.print(inches_Bottom_Left);
  Serial.println(" Inches at Bottom Left Sensor");
  delay(1000); 

}
