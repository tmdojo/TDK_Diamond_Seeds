#include <TDK_Diamond_Seeds.h>

// measurement takes about 6.5ms in Normal mode
// measurement takes about 25ms in Low Noise mode (default)

float Pa;

void setup() {

  TDSs.begin();

}

void loop() {

  // unit is in Pascals

  Pa=TDSs.Pressure();

  Serial.println(Pa);

  delay(100);

}
