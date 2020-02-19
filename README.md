# How to use TDK Diamond Seeds Arduino Library

This is Arduino library for TDK Diamond Seeds.
To use this library, the following library must be installed.
Please install the following libraries according to the installation.

## Installation

1. download the following 4 zip files
1. In the Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library and include the following 4 zip files

- [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel/archive/master.zip).
- [Adafruit ADS1015 Library](https://github.com/adafruit/Adafruit_ADS1X15/archive/master.zip).
- [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor/archive/master.zip).
- [DRV2667 Library](https://github.com/yurikleb/DRV2667/archive/master.zip).


## Functions

- TDSs.begin(void);
- TDSs.MagneticAngle(void);
- TDSs.MagneticAngleRadian(void);
- TDSs.Tesla(void);
- TDSs.AccelX(void);
- TDSs.AccelY(void);
- TDSs.AccelZ(void);
- TDSs.GyroX(void);
- TDSs.GyroY(void);
- TDSs.GyroZ(void);
- TDSs.Pressure(void);
- TDSs.Thermistor(void);
- TDSs.SoundPressure(void);
- TDSs.Filter(float input);
- TDSs.switchAnalog(void);
- TDSs.PlayWavFile(char wavfile[]);
- TDSs.PlayWave(byte WaveForm[][4], byte WavesNumber);
- TDSs.playTone(uint16_t freq, uint16_t time, boolean wait=true);
- TDSs.leftButton(void);
- TDSs.rightButton(void);
- TDSs.slideSwitch(void);
- TDSs.LEDColor(uint8_t pix, uint8_t red, uint8_t green, uint8_t blue);

## Examples
There are many examples implemented in this library. One of the examples is below. You can find other examples [here](https://github.com/tmdojo/TDK_Diamond_Seeds/tree/master/examples).

Hello_Pressure_sensor.ino

```C++
#include <TDK_Diamond_Seeds.h>

float Pa;

void setup() {
  TDSs.begin();
}

void loop() {
  // unit is in Pascals
  Pa=TDSs.Pressure();
  Serial.println(Pa);
  delay(100);
}
```
## License

TDK_Diamond_Seeds is free software: you can redistribute it and/or  modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
TDK_Diamond_Seeds is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [GNU Lesser General Public License](https://www.gnu.org/licenses/lgpl-3.0.en.html) for more details.
You should have received a copy of the GNU Lesser General Public License along with NeoPixel.  If not, see [this](https://www.gnu.org/licenses/)
