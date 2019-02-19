#include <TDK_Diamond_Seeds.h>

float Gx,Gy,Gz;

void setup() {
  TDSs.begin();
}

void loop() {

  Gx = TDSs.GyroX();
  Gy = TDSs.GyroY();
  Gz = TDSs.GyroZ();

  Serial.print("Gx: ");
  Serial.print(Gx);
  Serial.print(" ,");
  Serial.print("Gy: ");
  Serial.print(Gy);
  Serial.print(" ,");
  Serial.print("Gz: ");
  Serial.println(Gz);

  delay(100);

}
