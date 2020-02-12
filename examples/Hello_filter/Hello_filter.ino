#include <TDK_Diamond_Seeds.h>

float Pa;
float Pa_filtered;

void setup() {

  TDSs.begin();

}

void loop() {

  // unit is in Pascals

  Pa=TDSs.Pressure();
  Pa_filtered=TDSs.Filter(Pa);

  Serial.print("Pa: ");
  Serial.print(Pa);
  Serial.print(" ,");
  Serial.print("FilteredPa: ");
  Serial.println(Pa_filtered);

}
