#include <TDK_Diamond_Seeds.h>

float Ax,Ay,Az;

void setup() {
  TDSs.begin();
}

void loop() {

  // unit is in g (i.e. normalized to 9.80665F m/s^2, Earth's gravity)

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
