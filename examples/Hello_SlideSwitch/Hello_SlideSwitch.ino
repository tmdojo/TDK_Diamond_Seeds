#include <TDK_Diamond_Seeds.h>

bool slideSwitch;

void setup() {
  TDSs.begin();
}

void loop() {
  slideSwitch = TDSs.slideSwitch();

  Serial.print("Slide Switch: ");
  if (slideSwitch) {
    Serial.print("left");
  } else {
    Serial.print("right");
  }
  Serial.println();

  delay(1000);
}
