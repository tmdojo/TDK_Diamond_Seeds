#include <TDK_Diamond_Seeds.h>

float Pa;

void setup() {

  TDSs.begin();

}

void loop() {

  Pa=TDSs.GetPressure();

  Serial.print("Pressure ");
  Serial.println(Pa);

}
