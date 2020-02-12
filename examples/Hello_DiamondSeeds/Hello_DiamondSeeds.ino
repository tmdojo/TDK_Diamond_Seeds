#include <TDK_Diamond_Seeds.h>

boolean Check1,Check2,Check3,Check4,Check5,Check6,Check7,Check8,Check9,Check10,Check11,Check12;

float Angle_i,Angle_rad_i,Mdf_i;
float Angle,Angle_rad,Mdf;

bool slideSwitch_i;
bool slideSwitch;

float Gx_i,Gy_i,Gz_i;
float Gx,Gy,Gz;

float Tt_i;
float Tt;

bool leftButtonPressed;
bool rightButtonPressed;

float Pi_i,Ti_i;
float Pi,Ti;

float Ax_i,Ay_i,Az_i;
float Ax,Ay,Az;

float dB_i,dB;

float Pa_i,Pa;

void setup(void){
  TDSs.begin();

  Check1 = true;
  Check2 = true;
  Check3 = true;
  Check4 = true;
  Check5 = true;
  Check6 = true;
  Check7 = true;
  Check8 = true;
  Check9 = true;
  Check10= true;
  Check11= true;
  Check12= true;

//Puzzle1:increase magnetic power
  Mdf_i = TDSs.Tesla();
  while(Check1){
    Mdf = TDSs.Tesla();
    Serial.println(abs(Mdf-Mdf_i));
    if(abs(Mdf-Mdf_i)>20){
      TDSs.setPixelColor(0, 255,   0,   0);
      Serial.println("Puzzle1:use magnet");
      Check1 = false;
    }
  }

//Puzzle2:use gyrosensor
  Gx_i = TDSs.GyroX();
  Gy_i = TDSs.GyroY();
  Gz_i = TDSs.GyroZ();
  while(Check2){
    Gx = TDSs.GyroX();
    Gy = TDSs.GyroY();
    Gz = TDSs.GyroZ();
    if(abs(Gx+Gy+Gz-Gx_i-Gy_i-Gz_i)>200){
      TDSs.setPixelColor(1, 0,   255,   0);
      Check2 = false;
      Serial.println("Puzzle2:use gyrosensor");
    }
  }
//Puzzle3:use slideswitch
  slideSwitch_i = TDSs.slideSwitch();
  if (slideSwitch_i) {
    while(Check3){
      slideSwitch = TDSs.slideSwitch();
      if(!slideSwitch){
        TDSs.setPixelColor(2, 0, 0, 255);
        Check3 = false;
        Serial.println("Puzzle3:slide switch");
      }
    }
  } else {
    while(Check3){
      slideSwitch = TDSs.slideSwitch();
      if(slideSwitch){
        TDSs.setPixelColor(2, 0, 0, 255);
        Check3 = false;
        Serial.println("Puzzle3:slide switch");
      }
    }
  }

//Puzzle4:use thermistor
  Tt_i = TDSs.Thermistor();
  while(Check4){
    Tt = TDSs.Thermistor();
    Serial.println(abs(Tt-Tt_i));
    if(abs(Tt-Tt_i)>2){
      TDSs.setPixelColor(3, 255,   0,   0);
      Check4 = false;
      Serial.println("Puzzle4:use thermistor");
    }
  }

//Puzzle5:use MEMS microphone
  dB_i=TDSs.SoundPressure();
  while(Check5){
    dB=TDSs.SoundPressure();
    Serial.println(abs(dB-dB_i));
    if(abs(dB-dB_i)>7500){
      TDSs.setPixelColor(4, 0,   255,   0);
      Check5 = false;
      Serial.println("Puzzle5:use MEMSmicrophone");
    }
  }

//Puzzle6:use right button
  while(Check6){
    if(TDSs.rightButton()){
      TDSs.setPixelColor(6, 255,   0,   0);
      Check6 = false;
      Serial.println("Puzzle6:use rightButton");
    }
  }

//Puzzle7:use acceleration sensor
  Ax_i = TDSs.AccelX();
  Ay_i = TDSs.AccelY();
  Az_i = TDSs.AccelZ();
  while(Check7){
    Ax = TDSs.AccelX();
    Ay = TDSs.AccelY();
    Az = TDSs.AccelZ();
    Serial.println(abs(Ax+Ay+Az-Ax_i-Ay_i-Az_i));
    if(abs(Ax+Ay+Az-Ax_i-Ay_i-Az_i)>2){
      TDSs.setPixelColor(7, 0,   255,   0);
      Check7 = false;
    }
  }

//Puzzle8:use left button
  while(Check8){
    if(TDSs.leftButton()){
      TDSs.setPixelColor(8, 0,   0,   255);
      Check8 = false;
    }
  }

//Puzzle9:use pressure_sensor
  Pa_i=TDSs.Pressure();
  while(Check9){
    Pa=TDSs.Pressure();
    Serial.println(abs(Pa-Pa_i));
    if(abs(Pa-Pa_i)>0.05){
      TDSs.setPixelColor(9, 255,   0,   0);
      Check9 = false;
      Serial.println("Puzzle9:use Pressure sensor");
    }
  }

//Puzzle10:use TMR angle sensor
  Tesla_i = TDSs.Tesla();
  while(Check11){
    Tesla = TDSs.Tesla();
    Serial.println(abs(Tesla-Tesla_i));
    if(abs(Tesla-Tesla_i)>20){
      Angle_i = TDSs.MagneticAngle();
      while(Check10){
        Angle=TDSs.MagneticAngle();
        Serial.println(abs(Angle-Angle_i));
        if(abs(Angle-Angle_i)>180){
          TDSs.setPixelColor(10, 0,   255,   0);
          Check10 = false;
          Serial.println("Puzzle10:use TMR angle sensor");
        }
      }
      Check11 = false;
    }
  }

  TDSs.switchAnalog();

  TDSs.setPixelColor(11, 0,   0,   255);
  delay(100);
  char filename[] = "TDK.wav";
  TDSs.PlayWavFile(filename);
  TDSs.setPixelColor(5, 0,   0,   255);

  Serial.println("Clear!");


}

void loop(void)
{

  char filename1[] = "bpm_60.wav";

  TDSs.PlayWavFile(filename1);


  delay(100);
}
