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
sensors_event_t event;
bool switch_AS=false;

#define MODE_6AXIS 1 //1 equals I2C, use 0 for SPI
#define SAMPLE_DIV 0 //Sample rate = 1khz/(1+Sample_Div)
#define USE_BYPASS 1 // bypass the aux I2C lines, use the main one

/* Assign a unique ID to all snsors used with the adafruit standard*/
Adafruit_ICM20789_ACC_Unified accel = Adafruit_ICM20789_ACC_Unified(14, MODE_6AXIS, ICM20789_ACC_RANGE_16_G);
Adafruit_ICM20789_GYRO_Unified gyro = Adafruit_ICM20789_GYRO_Unified(28, MODE_6AXIS, ICM20789_GYRO_RANGE_2000_DPS);
Adafruit_ICM20789_BARO_Unified pressure = Adafruit_ICM20789_BARO_Unified(44, USE_BYPASS, MODE_6AXIS);


/**************************************************************************/
/*!
    @brief  Set up the CircuitPlayground hardware
    @param  brightness Optional brightness to set the neopixels to
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/

void TDK_DiamondSeeds::begin(void){

  pinMode(AUDIOPIN, OUTPUT);
  pinMode(THERMISTORPIN,INPUT);
  pinMode(LEFTBUTTON, INPUT_PULLDOWN);
  pinMode(RIGHTBUTTON, INPUT_PULLDOWN);
  pinMode(SLIDESWITCH, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(9600);
  configureICM20789(MODE_6AXIS, SAMPLE_DIV);
  startICM20789_6axis(MODE_6AXIS);
  delay(500);
  accel.begin();
  gyro.begin();
  pressure.begin();
  AudioZero_TDSs.begin(44100*2);
  AudioZero_TDSs.end();
  strip = Adafruit_NeoPixel();
  strip.updateType(NEO_GRB + NEO_KHZ800);
  strip.updateLength(12);
  strip.setPin(NEOPIXELPIN);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(20);

  // start I2S at 16 kHz with 32-bits per sample
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
  Serial.println("Failed to initialize I2S!");
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

  amp = sqrt(sq(sin_mv)+sq(cos_mv));
  if (amp < OUTPUT_AMP_1) {
    mfs = MAGNETIC_FIELD_0+(MAGNETIC_FIELD_1-MAGNETIC_FIELD_0)*(amp-OUTPUT_AMP_0)/(OUTPUT_AMP_1-OUTPUT_AMP_0);
  }else if (amp  < OUTPUT_AMP_2) {
    mfs = MAGNETIC_FIELD_1+(MAGNETIC_FIELD_2-MAGNETIC_FIELD_1)*(amp-OUTPUT_AMP_1)/(OUTPUT_AMP_2-OUTPUT_AMP_1);
  }else if (amp  < OUTPUT_AMP_3) {
    mfs = MAGNETIC_FIELD_2+(MAGNETIC_FIELD_3-MAGNETIC_FIELD_2)*(amp-OUTPUT_AMP_2)/(OUTPUT_AMP_3-OUTPUT_AMP_2);
  }else if (amp  < OUTPUT_AMP_4) {
    mfs = MAGNETIC_FIELD_3+(MAGNETIC_FIELD_4-MAGNETIC_FIELD_3)*(amp-OUTPUT_AMP_3)/(OUTPUT_AMP_4-OUTPUT_AMP_3);
  }else if (amp  >= OUTPUT_AMP_4) {
    mfs = MAGNETIC_FIELD_4;
  }

  return mfs;

}

float TDK_DiamondSeeds::AccelX(void){
  float ax;
  accel.getEvent(&event);
  ax = event.acceleration.x;
  return ax;
}

float TDK_DiamondSeeds::AccelY(void){
  float ay;
  accel.getEvent(&event);
  ay = event.acceleration.y;
  return ay;
}

float TDK_DiamondSeeds::AccelZ(void){
  float az;
  accel.getEvent(&event);
  az = event.acceleration.z;
  return az;
}

float TDK_DiamondSeeds::GyroX(void){
  float gx;
  gyro.getEvent(&event);
  gx = event.gyro.x;
  return gx;
}

float TDK_DiamondSeeds::GyroY(void){
  float gy;
  gyro.getEvent(&event);
  gy = event.gyro.y;
  return gy;
}

float TDK_DiamondSeeds::GyroZ(void){
  float gz;
  gyro.getEvent(&event);
  gz = event.gyro.z;
  return gz;
}

float TDK_DiamondSeeds::Pressure(void){
  float press;
  pressure.getEvent(&event);
  press = event.pressure;
  return press;
}

float TDK_DiamondSeeds::Filter(float input){
  float output;
  float coeff[TAP_LENGTH] {0.0071,
                           0.0101,
                           0.0179,
                           0.0303,
                           0.0459,
                           0.0629,
                           0.0792,
                           0.0927,
                           0.1016,
                           0.1046,
                           0.1016,
                           0.0927,
                           0.0792,
                           0.0629,
                           0.0459,
                           0.0303,
                           0.0179,
                           0.0101,
                           0.0071,
                           0.0000};

  for (int i=(TAP_LENGTH-1); i>0 ; i--) {
    buffer[i] = buffer[i-1];
  }
  buffer[0] = input;
  output = 0;
  for (int i=0; i<TAP_LENGTH ; i++) {
    output += buffer[i]*coeff[i];
  }
  return output;
}

void TDK_DiamondSeeds::switchAnalog(void){
  SD.begin(4);

  drv.setToAnalogInput();  //Swithch To Analog

}

void TDK_DiamondSeeds::PlayWavFile(char wavfile[]){

  AudioZero_TDSs.begin(44100*2);
  File myFile = SD.open(wavfile);
  Serial.print("Play:");
  Serial.println(wavfile);
  AudioZero_TDSs.play(myFile);
  AudioZero_TDSs.end();

}

void TDK_DiamondSeeds::PlayWave(byte WaveForm[][4], byte WavesNumber){

//    WaveForm[0][1]/=7.8125;
//    WaveForm[0][2]/=1000/WaveForm[0][1];

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
