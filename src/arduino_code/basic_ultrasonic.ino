/*
Code for ultrasonic distance measurements (HC-SR04) was adapted and modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
Goal is for there to be six ultrasonic sensors around the robot (2 on the left side, 2 on the right side, one on front, one on back)
and create distance calculations of knowing where the robot is relative to the gameboard using a coordinate system
*/

//Front Left trig and echo pin 
#define Front_Left_trigPin 4    // Trigger
#define Front_Left_echoPin 5    // Echo

//Back Left trig and echo pin
#define Back_Left_trigPin 3
#define Back_Left_echoPin 2

//Top Left trig and echo pin
#define Top_Left_trigPin 6
#define Top_Left_echoPin 7

//Front Right trig and echo pin
#define Front_Right_trigPin 9
#define Front_Right_echoPin 8

//Back Right trig and echo pin
#define Back_Right_trigPin 11
#define Back_Right_echoPin 10

//Bottom Right trig and echo pin
#define Bottom_Right_trigPin 13
#define Bottom_Right_echoPin 12
//Coordinate System data points below needed for M2
int starting_x, starting_y;
int current_x,current_y;
//bool rotation = false;

void setup() 
{
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
  //Front Right
  pinMode(Front_Right_trigPin,OUTPUT);
  pinMode(Front_Right_echoPin,INPUT);
  //Back Right
  pinMode(Back_Right_trigPin,OUTPUT);
  pinMode(Back_Right_echoPin,INPUT);
  //Bottom Right
  pinMode(Bottom_Right_trigPin,OUTPUT);
  pinMode(Bottom_Right_echoPin,INPUT);
}
 
void loop() 
{
  //for each ultrasonic sensor calculate the distance measurement in inches
  int FL = findFrontLeftDistance();
  int BL = findBackLeftDistance();
  int TL = findTopLeftDistance();
  int FR = findFrontRightDistance();
  int BaR = findBackRightDistance();
  int BoR = findBottomRightDistance();
  Serial.println(FL);
  Serial.println(BL);
  Serial.println(TL);
  Serial.println(FR);
  Serial.println(BaR);
  Serial.println(BoR);
  //now locate the robot based on the ultrasonic measurements to relative gameboard locations
  if((FL < 3) || (FR < 3))
  {
    if(FL<3)
    {
      Serial.println("Left Wall");
    }
    else
    {
      Serial.println("Right Wall");
    }
  }
  if((FL <3) &&(TL <3))
  {
    Serial.println("Top Left Corner");
  }
  if((BaR <3) &&(BoR <3))
  {
    Serial.println("Back Right Corner");
  }
  //Check the allignment of the robot
  alligned(FL,BL,FR,BaR);
  //check the orientation of the robot, if it vertical or horizontal
  orientation(FL,BL,FR,BaR,TL,BoR);
  //For M2
  //Now do the coordinate system:
 // get_startingpos();
  //get current position and relative to the starting position
 // get_current_position();
  
}

/*
//find new position of the robot relatice to the starting position
void get_current_position()
{
  current_x = get_current_x_pos();//find current positions
  current_y = get_current_y_pos();
//include a bounds check of the limits of the board
//84 is the x axis and dimension is 8ft and the robot is 1ft in dimension (8-1ft)
//21 is the y axis and dimension is 4ft and the robot is 1ft in dimension (4-1ft)
  if((current_x < 84)&&(current_y < 21))
  {
      current_x = current_x - starting_x;//find position relative to the starting point
      current_y = current_y - starting_y;
  }
 
}
void get_startingpos()
{
  //save the initial x and y measurements and save to the coordinate (0,0)
  starting_x = findFrontLeftDistance() + (RW/2);//RW = robot width must enter!
  starting_y = findTopLeftDistance() + (RW/2);//RW = robot width must enter!
}
*/
void orientation(int front_left,int back_left,int front_right,int back_right, int top_left, int bottom_right);
{//Determines the position of the robot and if it vertical or horizontal
    
  if((front_left||back_left||front_right||back_right) > (top_left||bottom_right))
  {
    Serial.println("Orientation is vertical");
  }
  else
  {
   Serial.println("Orientation is Horizontal"); 
  }
}
void alligned(int front_left,int back_left,int front_right,int back_right);
{
  if(front_left > back_left)
  {
    Serial.println("Robot is Diaginal Left: rotate front left CCW");
  }
  else if (back_left > front_left)
  {
   Serial.println("Robot is Diaginal Left: rotate back left CW"); 
  }
  else if(front_right > back_right)
  {
    Serial.println("Robot is Diaginal Right: rotate front right CW");
  }
  else if(back_right > front_right)
  {
    Serial.println("Robot is Diaginal Right: rotate back right CCW");
  }
  else
  {
    Serial.println("Robot is alligned");
  }
}

/*
//Now with rotations of the robot must rotate or switch the x and y values
//Doing 90 degrees rotation
void rotation()
{
  //1) Find current allignment
  //2) Rotate the current x and y measurements
  int temp;
  temp = current_x;
  current_x = current_y;
  current_y = temp;
  if(rotation = false)
  { 
    rotation = true;
  }
  else
  {
    rotation = false;
  }
}
*/
//for each of the sensor measurements followed reference listed in comment header
int findFrontLeftDistance()
{
  int duration_Front_Left,inches_Front_Left;
  digitalWrite(Front_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Left_trigPin, LOW);
  pinMode(Front_Left_echoPin, INPUT);
  duration_Front_Left = pulseIn(Front_Left_echoPin, HIGH);
  // Convert the time into a distance
  inches_Front_Left = (duration_Front_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
//  Serial.print(inches_Front_Left);
//  Serial.println(" Inches at Front Left Sensor");
  delay(1000);
  return inches_Front_Left; 

}
int findBackLeftDistance()
{
//below is back left calculations
  int duration_Back_Left,inches_Back_Left;
  digitalWrite(Back_Left_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Left_trigPin, LOW);
  pinMode(Back_Left_echoPin, INPUT);
  duration_Back_Left = pulseIn(Back_Left_echoPin, HIGH);
  inches_Back_Left = (duration_Back_Left/2) / 74;   // Divide by 74 or multiply by 0.0135
//  Serial.print(inches_Back_Left);
//  Serial.println(" Inches at Back Left Sensor");
  delay(1000);
  return inches_Back_Left; 
}
int findTopLeftDistance()
{
 //below is Top Left calculations
  int duration_Top_Left,inches_Top_Left;
  digitalWrite(Top_Left_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Top_Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Top_Left_trigPin,LOW);
  pinMode(Top_Left_echoPin, INPUT);
  duration_Top_Left = pulseIn(Top_Left_echoPin, HIGH);
  inches_Top_Left = (duration_Top_Left/2) / 74;  
//  Serial.print(inches_Top_Left);
//  Serial.println(" Inches at Top Left Sensor");
  delay(1000);
  return inches_Top_Left; 
}
int findFrontRightDistance()
{
  int duration_Front_Right,inches_Front_Right;
  digitalWrite(Front_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Front_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Right_trigPin,LOW);
  pinMode(Front_Right_echoPin, INPUT);
  duration_Front_Right = pulseIn(Front_Right_echoPin, HIGH);
  inches_Front_Right = (duration_Front_Right/2) / 74;  
//  Serial.print(inches_Front_Right);
//  Serial.println(" Inches at Front Right Sensor");
  delay(1000);
  return inches_Front_Right; 
}
int findBackRightDistance()
{
  int duration_Back_Right,inches_Back_Right;
  digitalWrite(Back_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Back_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Right_trigPin,LOW);
  pinMode(Back_Right_echoPin, INPUT);
  duration_Back_Right = pulseIn(Back_Right_echoPin, HIGH);
  inches_Back_Right = (duration_Back_Right/2) / 74;  
//  Serial.print(inches_Back_Right);
//  Serial.println(" Inches at Back Right Sensor");
  delay(1000);
  return inches_Back_Right; 
}

int findBottomRightDistance()
{
  int duration_Bottom_Right,inches_Bottom_Right;
  digitalWrite(Bottom_Right_trigPin,LOW);
  delayMicroseconds(5);
  digitalWrite(Bottom_Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Bottom_Right_trigPin,LOW);
  pinMode(Bottom_Right_echoPin, INPUT);
  duration_Bottom_Right = pulseIn(Bottom_Right_echoPin, HIGH);
  inches_Bottom_Right = (duration_Bottom_Right/2) / 74;  
//  Serial.print(inches_Bottom_Right);
//  Serial.println(" Inches at Bottom Right Sensor");
  delay(1000);
  return inches_Bottom_Right; 
}
