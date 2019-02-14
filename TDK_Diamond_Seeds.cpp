
// Note that this is a simple way to derive angle with limited accuracy
// TDK has developed elavorated correction algorithm to achieve higher accuracy.
// Please contact TDK for details.

/*!
 * @file Adafruit_CircuitPlayground.cpp
 *
 * @mainpage Adafruit CircuitPlayground Library
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's CircuitPlayground driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit CircuitPlayground boards:
 *  - https://www.adafruit.com/products/3000
 *  - https://www.adafruit.com/products/3333
 *
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * @section author Author
 *
 * Written by Ladyada and others for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "TDK_Diamond_Seeds.h"

Adafruit_ADS1115 ads(0x48);
Yurikleb_DRV2667 drv;
bool switch_AS=false;


#define MODE_6AXIS 1 //1 equals I2C, use 0 for SPI
#define SAMPLE_DIV 0 //Sample rate = 1khz/(1+Sample_Div)
#define USE_BYPASS 1 // bypass the aux I2C lines, use the main one

/* Assign a unique ID to all snsors used with the adafruit standard*/
Adafruit_ICM20789_ACC_Unified accel = Adafruit_ICM20789_ACC_Unified(14, MODE_6AXIS, ICM20789_ACC_RANGE_2_G);
Adafruit_ICM20789_GYRO_Unified gyro = Adafruit_ICM20789_GYRO_Unified(28, MODE_6AXIS, ICM20789_GYRO_RANGE_250_DPS);
Adafruit_ICM20789_BARO_Unified pressure = Adafruit_ICM20789_BARO_Unified(44, USE_BYPASS, MODE_6AXIS);

/**************************************************************************/
/*!
    @brief  Set up the CircuitPlayground hardware
    @param  brightness Optional brightness to set the neopixels to
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/

void TDK_DiamondSeeds::begin(void){
  Wire.begin();
  Serial.begin(9600);
  ads.begin();
  configureICM20789(MODE_6AXIS, SAMPLE_DIV);
  startICM20789_6axis(MODE_6AXIS);
  delay(500);
  accel.begin();
  gyro.begin();
  pressure.begin();
  drv.begin();

  pinMode(AUDIOPIN, OUTPUT);
  pinMode(THERMISTORPIN,INPUT);
  pinMode(LEFTBUTTON, INPUT_PULLDOWN);
  pinMode(RIGHTBUTTON, INPUT_PULLDOWN);
  pinMode(SLIDESWITCH, INPUT_PULLUP);

  strip = Adafruit_CPlay_NeoPixel();
  strip.updateType(NEO_GRB + NEO_KHZ800);
  strip.updateLength(12);
  strip.setPin(NEOPIXELPIN);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(20);

  // start I2S at 16 kHz with 32-bits per sample
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
  //Serial.println("Failed to initialize I2S!");
  while (1); // do nothing
  }
}

float TDK_DiamondSeeds::MagneticAngle(void){

  results0_1 = ads.readADC_Differential_0_1();
  results2_3 = ads.readADC_Differential_2_3();

  sin_mv = results0_1;
  cos_mv = results2_3;

  angle_rad = atan2(sin_mv, cos_mv);
  angle = angle_rad * rad2deg_multiplier; // convert radian to degree

  return angle;
}

float TDK_DiamondSeeds::MagneticAngleRadian(void){

  results0_1 = ads.readADC_Differential_0_1();
  results2_3 = ads.readADC_Differential_2_3();

  sin_mv = results0_1;
  cos_mv = results2_3;

  angle_rad = atan2(sin_mv, cos_mv);

  return angle_rad;
}

float TDK_DiamondSeeds::Tesla(void){

  results0_1 = ads.readADC_Differential_0_1();
  results2_3 = ads.readADC_Differential_2_3();

  sin_mv = results0_1;
  cos_mv = results2_3;

  rho = sqrt(sq(sin_mv)+sq(cos_mv));

  return rho/100;

}

float TDK_DiamondSeeds::AccelX(void){
  float ax;
  ax = accel.getX()/100;
  return ax;
}

float TDK_DiamondSeeds::AccelY(void){
  float ay;
  ay = accel.getY()/100;
  return ay;
}

float TDK_DiamondSeeds::AccelZ(void){
  float az;
  az = accel.getZ()/100;
  return az;
}

float TDK_DiamondSeeds::GyroX(void){
  float gx;
  gx = gyro.getX()/100;
  return gx;
}

float TDK_DiamondSeeds::GyroY(void){
  float gy;
  gy = gyro.getY()/100;
  return gy;
}

float TDK_DiamondSeeds::GyroZ(void){
  float gz;
  gz = gyro.getZ()/100;
  return gz;
}

float TDK_DiamondSeeds::GetPressure(void){
  float press;
  press = pressure.getPress();
  return press;
}

float TDK_DiamondSeeds::GetTemperature(void){
  float Temp;
  Temp = pressure.getTemp();
  return Temp;
}

void TDK_DiamondSeeds::switchAnalog(void){

  delay(100);

// setup SD-card
  if (SD.begin(4)) {

    // 44100kHz stereo => 88200 sample rate
    AudioZero.begin(44100*2);
    switch_AS=true;

  }

  //drv.begin();
  drv.setToAnalogInput();  //Swithch To Analog

}

void TDK_DiamondSeeds::PlayWavFile(char wavfile[]){

  File myFile = SD.open(wavfile);
  Serial.print("Play:");
  Serial.println(wavfile);
  AudioZero.play(myFile);

}

void TDK_DiamondSeeds::PlayWave(byte WaveForm[][4], byte WavesNumber){

    drv.playWave(WaveForm, sizeof(WaveForm)); //Play one the Waveforms defined above;

}

void TDK_DiamondSeeds::playTone(uint16_t freq, uint16_t time, boolean wait) {
  tone(AUDIOPIN, freq, time);
  if (wait) delay(time);
}

float TDK_DiamondSeeds::Thermistor(void) {
   // Thermistor test
  float reading;

  reading = analogRead(THERMISTORPIN);

  //Serial.print("Thermistor reading: "); Serial.println(reading);

  // convert the value to resistance
  reading = ((1023.0 * SERIESRESISTOR) / reading);
  reading -= SERIESRESISTOR;

  //Serial.print("Thermistor resistance: "); Serial.println(reading);

  float steinhart;
  steinhart = reading / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}

uint32_t TDK_DiamondSeeds::colorWheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

boolean TDK_DiamondSeeds::slideSwitch(void) {
  return digitalRead(SLIDESWITCH);
}

boolean TDK_DiamondSeeds::leftButton(void) {
  return digitalRead(LEFTBUTTON);
}

boolean TDK_DiamondSeeds::rightButton(void) {
  return digitalRead(RIGHTBUTTON);
}

#define P_SAMPLES 128

float TDK_DiamondSeeds::SoundPressure(void) {
  // read a bunch of samples:
int samples[P_SAMPLES];

for (int i=0; i<P_SAMPLES; i++) {
  int sample = 0;
  //while ((sample == 0) || (sample == -1) ) {
    sample = I2S.read();
    //Serial.print(sample);
  //}
  // convert to 18 bit signed
  sample >>= 14;
  samples[i] = sample;

}

// ok we hvae the samples, get the mean (avg)
float meanval = 0;
for (int i=0; i<P_SAMPLES; i++) {
  meanval += samples[i];
}
meanval /= P_SAMPLES;
//Serial.print("# average: " ); Serial.println(meanval);

// subtract it from all sapmles to get a 'normalized' output
for (int i=0; i<P_SAMPLES; i++) {
  samples[i] -= meanval;
  //Serial.println(samples[i]);
}

// find the 'peak to peak' max
float maxsample, minsample;
minsample = 100000;
maxsample = -100000;
for (int i=0; i<P_SAMPLES; i++) {
  minsample = min(minsample, samples[i]);
  maxsample = max(maxsample, samples[i]);
}

float SoundDiff = maxsample - minsample;

  return SoundDiff;

}

// instantiate static
TDK_DiamondSeeds TDSs;
