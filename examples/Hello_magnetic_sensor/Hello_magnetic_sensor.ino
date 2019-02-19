#include <TDK_Diamond_Seeds.h>

float Angle,Angle_rad,Tesla;

void setup(void)
{
  TDSs.begin();
}

void loop(void)
{
  Angle = TDSs.MagneticAngle();
  Angle_rad = TDSs.MagneticAngleRadian();
  Tesla = TDSs.Tesla();

  Serial.print("Angle: ");
  Serial.print(Angle);
  Serial.print(" ,");
  Serial.print("Angle(rad): ");
  Serial.print(Angle_rad);
  Serial.print(" ,");
  Serial.print("Tesla: ");
  Serial.println(Tesla);

  delay(100);
}
