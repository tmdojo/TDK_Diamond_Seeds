#include "TDK_Diamond_Seeds.h"

float Tt;

void setup() {
  TDSs.begin();
}


void loop()
{

  Tt = TDSs.Thermistor();

  Serial.print("temperature ");
  Serial.println(Tt);

  delay(100);

}
