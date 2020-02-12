#include <TDK_Diamond_Seeds.h>

float Angle,Angle_rad,Mdf;

void setup(void)
{
  TDSs.begin();
}

void loop(void)
{

    // Angle unit is in Dgrees
    // Angle_rad unit is in rad
    // Mdf unit is in mT

  Angle = TDSs.MagneticAngle();
  Angle_rad = TDSs.MagneticAngleRadian();
  Mdf = TDSs.Tesla();

  Serial.print("Angle: ");
  Serial.print(Angle);
  Serial.print(" ,");
  Serial.print("Angle(rad): ");
  Serial.print(Angle_rad);
  Serial.print(" ,");
  Serial.print("Tesla: ");
  Serial.println(Mdf);

  delay(100);
}
