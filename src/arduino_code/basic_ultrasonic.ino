int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
int trigPinS = 10;
int echoPinS = 9;
int trigPinT = 8;
int echoPinT = 13;
long duration, cm, inches;
long duration1, cm1, inches1;
long durationT,cmT,inchesT;
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinS, OUTPUT);
  pinMode(echoPinS, INPUT);
  pinMode(trigPinT,INPUT);
  pinMode(echoPinT,OUTPUT);
}
 
void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135

  //Serial.print(inches);
  //Serial.print("in FRONT, ");
  //Serial.println();

  delay(100);

  digitalWrite(trigPinS, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinS, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinS, LOW);

  pinMode(echoPinS, INPUT);

  duration1 = pulseIn(echoPinS, HIGH);
 
  // Convert the time into a distance
  cm1 = (duration1/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches1 = (duration1/2) / 74;   // Divide by 74 or multiply by 0.0135

  //Serial.print(inches1);
  //Serial.print("in SIDE, ");
  //Serial.println();
  delay(100);
  digitalWrite(trigPinT,LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinT, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinT,LOW);
  
  pinMode(echoPinT, INPUT);

  durationT = pulseIn(echoPinT, HIGH);
  cmT = (durationT/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inchesT = (durationT/2) / 74;  

  delay(100);

  if(inches <= 3)
  {
    if(inches1 <= 3)
    {
      Serial.print("CORNER Detected");
      Serial.println();
    }
    else
    {
    Serial.print("On Gameboard");
    Serial.println();
    }
  }
  else if(inches1 <= 3)
  {
    Serial.print("SIDE Detected");
    Serial.println();
  }
  else if(inchesT-inches1 <= 2)
  {
      Serial.print("MIDDLE");
      Serial.println();
    
  }
 else
  {
    Serial.print("On Gameboard");
    Serial.println();
    
  }
 

  delay(250);

}
