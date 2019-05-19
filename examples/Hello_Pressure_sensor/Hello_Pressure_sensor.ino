#include <TDK_Diamond_Seeds.h>

float Pa;

void setup() {

  TDSs.begin();

}

void loop() {

  // measurement takes about 6.5ms in Normal mode
  // measurement takes about 25ms in Low Noise mode (default)
  Pa=TDSs.GetPressure();

  // GetPressure returns in HectoPascals
  //Serial.print("Pressure ");
  //Serial.println(Pa);

  // Open serial plotter to check reading
  // it's more obvious to see effect in Pascals
  Serial.println(Pa*100);
}
