#include <TDK_Diamond_Seeds.h>

bool leftButtonPressed;
bool rightButtonPressed;

void setup() {
  TDSs.begin();
}

void loop() {
  leftButtonPressed = TDSs.leftButton();
  rightButtonPressed = TDSs.rightButton();

  Serial.print("Left Button: ");
  if (leftButtonPressed) {
    Serial.print("DOWN");
  } else {
    Serial.print("  UP");
  }
  Serial.print("   Right Button: ");
  if (rightButtonPressed) {
    Serial.print("DOWN");
  } else {
    Serial.print("  UP");
  }
  Serial.println();

  delay(1000);
}
