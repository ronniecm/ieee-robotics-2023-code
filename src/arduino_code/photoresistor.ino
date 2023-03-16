//Link for wiring: https://www.instructables.com/How-to-use-a-photoresistor-or-photocell-Arduino-Tu/
//(only need photoresistor, 10k resistor, A0 pin, 5V, ground)
//Constants
int pResistor = A0; // Photoresistor at Arduino analog pin A0
//int redLED = 2;


void setup()
{
  Serial.begin(9600);
  pinMode(pResistor, INPUT);// Set pResistor - A0 pin as an input (optional)
  pinMode(redLED, OUTPUT);
}
// range for red detection is 750-815
void loop(){
  //digitalWrite(redLED,HIGH);//For testing
  int value = analogRead(pResistor);
  // Serial.println(value);
  if (value >750 && value <820)
  {
    Serial.println("START");
  }
  
  delay(100);

}