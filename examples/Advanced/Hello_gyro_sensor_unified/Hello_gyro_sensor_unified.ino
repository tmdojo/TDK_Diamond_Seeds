// Install Adafruit Unified Sensor Driver
// https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_Sensor.h>

#include <TDK_Diamond_Seeds.h>

void setup() {
  TDSs.begin();
}

void loop() {
  // sensors_event_t is defined in Adafruit_Sensor.h
  sensors_event_t event;
  TDSs.GyroEvent(&event);

  // Gx,Gy,Gz in rad/s unit
  Serial.print(event.gyro.x);
  Serial.print(",");
  Serial.print(event.gyro.y);
  Serial.print(",");
  Serial.print(event.gyro.z);
  Serial.println("");

  delay(100);

}
