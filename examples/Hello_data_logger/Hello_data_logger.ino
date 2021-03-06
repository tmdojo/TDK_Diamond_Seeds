#include <TDK_Diamond_Seeds.h>

unsigned long tm;
float Ax,Ay,Az;
float Gx,Gy,Gz;
float Pa;
float dB;
float Angle,Angle_rad,Mfd;
float Tt;

File Logfile;

void setup() {
  int CardSelect = 4;
  TDSs.begin();
  if (!SD.begin(CardSelect)) {
    return;
  }
  char filename[16];
  strcpy(filename, "/OUTPUT00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0'+i/10;
    filename[8] = '0'+i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }
  Logfile = SD.open(filename, FILE_WRITE);
  if( ! Logfile ) {
    return;
  }

 // unit is in g (i.e. normalized to 9.80665F m/s^2, Earth's gravity)

  Logfile.print("time[ms]");
  Logfile.print(",");
  Logfile.print("Gx[deg/s]");
  Logfile.print(" ,");
  Logfile.print("Gy[deg/s]");
  Logfile.print(" ,");
  Logfile.print("Gz[deg/s]");
  Logfile.print(" ,");
  Logfile.print("Ax[g]");
  Logfile.print(" ,");
  Logfile.print("Ay[g]");
  Logfile.print(" ,");
  Logfile.print("Az[g]");
  Logfile.print(" ,");
  Logfile.print("Pa[Pa]");
  Logfile.print(" ,");
  Logfile.print("dB[a.u]");
  Logfile.print(" ,");
  Logfile.print("Angle[deg]");
  Logfile.print(" ,");
  Logfile.print("Angle_rad[rad]");
  Logfile.print(" ,");
  Logfile.print("Mfd[mT]");
  Logfile.print(" ,");
  Logfile.println("T[degC]");
  Logfile.flush();


}

void loop() {

  tm = millis();
  Gx = TDSs.GyroX();
  Gy = TDSs.GyroY();
  Gz = TDSs.GyroZ();
  Ax = TDSs.AccelX();
  Ay = TDSs.AccelY();
  Az = TDSs.AccelZ();
  Pa = TDSs.GetPressure()*100;
  dB = TDSs.SoundPressure();
  Angle = TDSs.MagneticAngle();
  Angle_rad = TDSs.MagneticAngleRadian();
  Mfd = TDSs.Tesla();
  Tt = TDSs.Thermistor();

  Serial.print("Gx: ");
  Serial.print(Gx);
  Serial.print(" ,");
  Serial.print("Gy: ");
  Serial.print(Gy);
  Serial.print(" ,");
  Serial.print("Gz: ");
  Serial.print(Gz);
  Serial.print(" ,");
  Serial.print("Ax: ");
  Serial.print(Ax);
  Serial.print(" ,");
  Serial.print("Ay: ");
  Serial.print(Ay);
  Serial.print(" ,");
  Serial.print("Az: ");
  Serial.print(Az);
  Serial.print(" ,");
  Serial.print("Pa: ");
  Serial.print(Pa);
  Serial.print(" ,");
  Serial.print("SPL: ");
  Serial.print(dB);
  Serial.print(" ,");
  Serial.print("Angle: ");
  Serial.print(Angle);
  Serial.print(" ,");
  Serial.print("Angle_rad: ");
  Serial.print(Angle_rad);
  Serial.print(" ,");
  Serial.print("Tesla: ");
  Serial.print(Mfd);
  Serial.print(" ,");
  Serial.print("temperature: ");
  Serial.println(Tt);

  Logfile.print(tm);
  Logfile.print(" ,");
  Logfile.print(Gx);
  Logfile.print(" ,");
  Logfile.print(Gy);
  Logfile.print(" ,");
  Logfile.print(Gz);
  Logfile.print(" ,");
  Logfile.print(Ax);
  Logfile.print(" ,");
  Logfile.print(Ay);
  Logfile.print(" ,");
  Logfile.print(Az);
  Logfile.print(" ,");
  Logfile.print(Pa);
  Logfile.print(" ,");
  Logfile.print(dB);
  Logfile.print(" ,");
  Logfile.print(Angle);
  Logfile.print(" ,");
  Logfile.print(Angle_rad);
  Logfile.print(" ,");
  Logfile.print(Mfd);
  Logfile.print(" ,");
  Logfile.println(Tt);
  Logfile.flush();
  delay(100);

}
