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
  /* Testing 
    <1cm: 960
    1cm:  938
    2cm:  846
    3cm:  790
    4cm:  735
    5cm:  727
  */
  //750 to 820
  
  if (pValue >720 && pValue <960)
  {
    Serial.println("START");
  }
  
  delay(100);

}
