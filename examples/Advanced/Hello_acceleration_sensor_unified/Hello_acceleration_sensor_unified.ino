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
  TDSs.AccelEvent(&event);

  // Ax,Ay,Az in m/s^2 unit
  Serial.print(event.acceleration.x);
  Serial.print(",");
  Serial.print(event.acceleration.y);
  Serial.print(",");
  Serial.print(event.acceleration.z);
  Serial.println("");

  delay(100);
}
