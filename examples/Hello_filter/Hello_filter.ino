#include <TDK_Diamond_Seeds.h>

float Pa;

void setup() {

  TDSs.begin();

}

void loop() {
  float Pa_filtered;
  // measurement takes about 6.5ms in Normal mode
  // measurement takes about 25ms in Low Noise mode (default)
  Pa=TDSs.GetPressure();
  Pa_filtered=TDSs.Filter(Pa);
  // GetPressure returns in HectoPascals
  //Serial.print("Pressure ");
  //Serial.println(Pa);

  // Open serial plotter to check reading
  // it's more obvious to see effect in Pascals
  Serial.print("Pa: ");
  Serial.print(Pa*100);
  Serial.print(" ,");
  Serial.print("FilteredPa: ");
  Serial.println(Pa_filtered*100);
  
}
