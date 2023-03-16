//Link for wiring: https://www.instructables.com/How-to-use-a-photoresistor-or-photocell-Arduino-Tu/
//(only need photoresistor, 10k resistor, A0 pin, 5V, ground)

//Note: The Red LED must be directly above the photoresistor!
int pResistor = A0; // Photoresistor at Arduino analog pin A0
void setup()
{
  Serial.begin(9600);
  pinMode(pResistor, INPUT);
}

void loop()
{
  int pValue = analogRead(pResistor);
  //Serial.println(pValue);
  //Analog values for red color range below 
  /* Testing Distance: Average Analog Reading
    1cm:  938
    2cm:  846
    3cm:  790
    4cm:  735
    5cm:  727
  */  
  if (pValue >720 && pValue <940)
  {
    Serial.println("START");
  }
  
  delay(100);

}
