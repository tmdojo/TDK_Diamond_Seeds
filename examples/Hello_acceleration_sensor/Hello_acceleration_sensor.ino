#include <TDK_Diamond_Seeds.h>

float Ax,Ay,Az;

void setup() {
  TDSs.begin();
}

void loop() {

  // unit is m/s^2

  Ax = TDSs.AccelX();
  Ay = TDSs.AccelY();
  Az = TDSs.AccelZ();

  Serial.print("Ax: ");
  Serial.print(Ax);
  Serial.print(" ,");
  Serial.print("Ay: ");
  Serial.print(Ay);
  Serial.print(" ,");
  Serial.print("Az: ");
  Serial.println(Az);

  delay(100);

}
