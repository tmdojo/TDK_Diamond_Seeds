#include <TDK_Diamond_Seeds.h>

float dB;

void setup() {
  TDSs.begin();
}

void loop() {

  // unit is in SoundPressureLevel[a.u.]

   dB = TDSs.SoundPressure();

  Serial.print("Sound Sensor SPL: ");
  Serial.println(dB);

  delay(10);
}
