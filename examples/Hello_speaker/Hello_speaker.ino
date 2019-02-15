#include <TDK_Diamond_Seeds.h>

void setup() {
  TDSs.begin();
}

void loop() {
  TDSs.playTone(2000, 100);
  Serial.println("play");
  delay(1000);
}
