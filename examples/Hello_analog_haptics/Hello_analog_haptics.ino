#include <TDK_Diamond_Seeds.h>
//wav file have to save 88200Hz, monaural,8bit
// Refer DS Wiki for how-to
// https://github.com/tmdojo/TDK_Diamond_Seeds/wiki/How-to-convert-audio-file-to-play-on-DS

void setup()
{
  TDSs.begin();
  TDSs.switchAnalog();
}

void loop(){

  char filename[] = "TDK.wav";
  char filename1[] = "bpm_60.wav";
  char filename2[] = "bpm_70.wav";
  char filename3[] = "bpm_80.wav";
  char filename4[] = "bpm_90.wav";
  char filename5[] = "bpm_100.wav";
  char filename6[] = "bpm_110.wav";
  char filename7[] = "bpm_120.wav";

  TDSs.PlayWavFile(filename);

  delay(100);

}
