#include <Wire.h>
#include "dispensingdropoff.h"

Dispensing dispense;

uint32_t startTime;
bool droplocationfound = false;


void setup(void)
{
  dispense.setup_bot();
}
void loop(void) {

  startTime = millis();
  while (!droplocationfound && (millis() - startTime < 2000))
  {
      if (dispense.checkcolor())
      {
        dispense.halt();
        droplocationfound = true;
      }
      else
      {
        dispense.right(100);
        delay(100);
      }
      Serial.print("a");
  }
  if (!droplocationfound)
  {
    if (dispense.checkcolor())
    {
      dispense.halt();
      droplocationfound = true;
    }
    else
    {
      dispense.forward(100);
      delay(500);
    }
    Serial.print("b");
  }
  startTime = millis();
  while (!droplocationfound && (millis() - startTime < 2000))
  {
     if (dispense.checkcolor())
     {
       dispense.halt();
       droplocationfound = true;
     }
     else
     {
       dispense.left(100);
       delay(100);
     }
     Serial.print("c");
  }
  if (!droplocationfound)
  {
    if (dispense.checkcolor())
    {
      dispense.halt();
      droplocationfound = true;
    }
    else
    {
      dispense.forward(100);
      delay(500);
    }
    Serial.print("d");
  }

  
}
