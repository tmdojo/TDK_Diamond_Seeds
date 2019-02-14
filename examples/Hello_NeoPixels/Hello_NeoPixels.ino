#include <TDK_Diamond_Seeds.h>

void setup() {
  TDSs.begin();
}

void loop() {
  TDSs.clearPixels();

  delay(500);

  TDSs.setPixelColor(0, 255,   0,   0);
  TDSs.setPixelColor(1,   0, 255,   0);
  TDSs.setPixelColor(2,   0,   0, 255);
  TDSs.setPixelColor(3, 255,   0,   0);
  TDSs.setPixelColor(4,   0, 255,   0);
  TDSs.setPixelColor(5,   0,   0, 255);

  TDSs.setPixelColor(6, 0xFF0000);
  TDSs.setPixelColor(7, 0x00FF00);
  TDSs.setPixelColor(8, 0x0000FF);
  TDSs.setPixelColor(9, 0xFF0000);
  TDSs.setPixelColor(10, 0x00FF00);
  TDSs.setPixelColor(11, 0x0000FF);

  delay(5000);
}
