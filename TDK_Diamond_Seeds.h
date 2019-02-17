/*!
 * @file Adafruit_Circuit_Playground.h
 *
 * This is part of Adafruit's CircuitPlayground driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit CircuitPlayground boards.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada and others for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef _TDK_DIAMOND_SEEDS_H
#define _TDK_DIAMOND_SEEDS_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "utility/Adafruit_ADS1015.h"
#include "utility/Adafruit_ICM20789_U.h"
#include "utility/Yurikleb_DRV2667.h"
#include "utility/AudioZero.h"
#include "utility/Adafruit_CPlay_NeoPixel.h"
#include "utility/Adafruit_Sensor.h"

#include "utility/I2S.h"

#define AUDIOPIN 14

#define SERIESRESISTOR 10000 ///< series resistor for thermistor
#define THERMISTORNOMINAL 10000 ///< resistance of thermistor at 25 degrees C
#define TEMPERATURENOMINAL 25 ///< temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT 3380 ///< The beta coefficient of the thermistor (usually 3000-4000)
#define THERMISTORPIN 19 ///<
#define NEOPIXELPIN 8 ///< neopixel pin
#define LEFTBUTTON 5  ///< left button pin
#define RIGHTBUTTON 6 ///< right button pin
#define SLIDESWITCH 10 ///< slide switch pin

/*!
  @brief Configuration to tune the color sensing logic:
   Amount of time (in milliseconds) to wait between
   changing the pixel color and reading the light
    sensor.
*/


/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with CircuitPlayground hardware
*/
/**************************************************************************/
class TDK_DiamondSeeds {

  public:

    void begin(void);
    float MagneticAngle(void);
    float MagneticAngleRadian(void);
    float Tesla(void);
    float AccelX(void);
    float AccelY(void);
    float AccelZ(void);
    float GyroX(void);
    float GyroY(void);
    float GyroZ(void);
    float GetPressure(void);
    float GetTemperature(void);
    void switchAnalog(void);
    void beginDigital(void);
    void beginBuzzer(void);
    void PlayWavFile(char wavfile[]);
    void PlayWave(byte WaveForm[][4], byte WavesNumber);
    void playTone(uint16_t freq, uint16_t time, boolean wait=true);
    float Thermistor(void);

    Adafruit_CPlay_NeoPixel strip; ///< the neopixel strip object
    //turn off all neopixels on the board
    void clearPixels(void) { strip.clear(); strip.show(); }
    void setPixelColor(uint8_t p, uint32_t c) {strip.setPixelColor(p, c); strip.show();}
    void setPixelColor(uint8_t p, uint8_t r, uint8_t g, uint8_t b) {strip.setPixelColor(p, r, g, b); strip.show();}
    void setBrightness(uint16_t b){strip.setBrightness(b);}
    uint8_t sine8(uint8_t x) { return strip.sine8(x); }
    uint8_t gamma8(uint8_t x) { return strip.gamma8(x); }
    uint32_t colorWheel(uint8_t x);

    boolean leftButton(void);
    boolean rightButton(void);
    boolean slideSwitch(void);

    float SoundPressure(void);

  private:
    //float multiplier; // convert ADC reading to mV
    const double rad2deg_multiplier = 180 / M_PI;
    int16_t results0_1, results2_3;
    float sin_mv;
    float cos_mv;
    float angle_rad;
    float angle;
    float rho;

};

extern TDK_DiamondSeeds TDSs; ///< instantiated by default

#endif
