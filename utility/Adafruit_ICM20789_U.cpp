/**************************************************************************/
/*!
    @file     Adafruit_ICM20789.cpp
    @author   Blast545

    To work with the current adafruit sensor implementation, the driver has to
    separate the three sensors: Gyroscope, Accelerometer, and Barometric Pressure
    with different drivers:

    Adafruit_ICM20789_ACC_Unified
    Adafruit_ICM20789_GYRO_Unified
    Adafruit_ICM20789_BARO_Unified

    v1.0  - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Adafruit_ICM20789_U.h"

// Read and write registers are not made part of the class,
// so these can be used manually without class objects,
// and to allow reuse with the three diferent drivers

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t readRegisterICM20789(uint8_t mode, uint8_t reg) {
  if (mode) {
    Wire.beginTransmission(ICM20789_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(ICM20789_I2C_ADDRESS, 1);
    return (Wire.read());
  }
  /* SPI method to be implemented:
  else {
    reg |= 0x80; // read byte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint8_t reply = spixfer(_clk, _di, _do, 0xFF);
    digitalWrite(_cs, HIGH);
    return reply;
  }
  */
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void writeRegisterICM20789(uint8_t mode, uint8_t reg, uint8_t value) {
  if (mode) {
    Wire.beginTransmission(ICM20789_I2C_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)(value));
    Wire.endTransmission();
  }
  /*
  else {
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    spixfer(_clk, _di, _do, value);
    digitalWrite(_cs, HIGH);
  }
  */
}

/**************************************************************************/
/*!
    @brief  Resets the ICM20789 to default state, starts required peripherals
    common for accel and gyro sensors
*/
/**************************************************************************/
void configureICM20789(uint8_t mode, uint8_t sample_rate_div) {

  // Based on the mode, init the required peripheral
  if (mode)
    Wire.begin();
  else {
    SPI.begin();
  }

  // Reset + leave power mode set to sleep
  writeRegisterICM20789(mode, MPUREG_PWR_MGMT_1, 0xC0);
  delay(50);

  // Disable FIFO
  // (Do nothing, disabled by default)

  // Set SAMPLE RATE: sampleRate = 1khz/(1+SMPLRT_DIV)
  // This sample rate is the rate at which the internal registers are updated (new info available)
  writeRegisterICM20789(mode, MPUREG_SMPLRT_DIV, sample_rate_div);
}

/**************************************************************************/
/*!
    @brief  Turns on the gyroscope and accelerometer sensor
*/
/**************************************************************************/
void startICM20789_6axis(uint8_t mode) {
  // Turns on the sensor
  writeRegisterICM20789(mode, MPUREG_PWR_MGMT_1, 0x00);
}

/**************************************************************************/
/*!
    @brief  Turns off the gyroscope and accelerometer sensor
*/
/**************************************************************************/
void stopICM20789_6axis(uint8_t mode) {
  // Turns on the sensor
  writeRegisterICM20789(mode, MPUREG_PWR_MGMT_1, 0x40);
}

/**************************************************************************/
/**************************************************************************/
/************** ACCELEROMETER functions and library ***********************/
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new ICM20789_ACC class
*/
/**************************************************************************/
Adafruit_ICM20789_ACC_Unified::Adafruit_ICM20789_ACC_Unified(int32_t sensorID, uint8_t mode, range_acc_icm_t rangeConfig) {
  _sensorID = sensorID;
  _i2c = mode;
  _range = rangeConfig;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ICM20789_ACC_Unified::begin() {

  // This start is in configureICM20789
  if (_i2c)
    Wire.begin();
  else {
    SPI.begin();
  }

  // Set accel scale
  setRange(_range);

  // Update internal offset value, registers X,Y,and Z, after updating the
  /*
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_L);
  _sensorOfssetX = aux2 | aux1<<8;

  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_L);
  _sensorOfssetY = aux2 | aux1<<8;

  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_L);
  _sensorOfssetZ = aux2 | aux1<<8;

  Serial.println(_sensorOfssetX, HEX);
  Serial.println(_sensorOfssetY, HEX);
  Serial.println(_sensorOfssetZ, HEX);
  */

  // Print Who I am register, and PWR_MGT_1 register to check comm is working
  //Serial.println(readRegisterICM20789(_i2c, MPUREG_WHO_AM_I), HEX);
  //Serial.println(readRegisterICM20789(_i2c, MPUREG_PWR_MGMT_1), HEX);

  return true;
}

/**************************************************************************/
/*!
    @brief  gets the current range value (from the class, not by reading)
*/
/**************************************************************************/
range_acc_icm_t Adafruit_ICM20789_ACC_Unified::getRange() {
  return _range;
}

/**************************************************************************/
/*!
    @brief  Sets a new range value
*/
/**************************************************************************/
void Adafruit_ICM20789_ACC_Unified::setRange(range_acc_icm_t range) {
  writeRegisterICM20789(_i2c, MPUREG_ACCEL_CONFIG, (_range<<3) );

  // Based on the selected range, set a different internal multiplier
  if(range==ICM20789_ACC_RANGE_2_G){
    _internalMultiplier =  (SENSORS_GRAVITY_STANDARD * 2)/32768;
  }

  else if(range==ICM20789_ACC_RANGE_4_G){
    _internalMultiplier = (SENSORS_GRAVITY_STANDARD * 4)/32768;
  }

  else if(range==ICM20789_ACC_RANGE_8_G){
    _internalMultiplier = (SENSORS_GRAVITY_STANDARD * 8)/32768;
  }

  else if(range==ICM20789_ACC_RANGE_16_G){
    _internalMultiplier = (SENSORS_GRAVITY_STANDARD * 16)/32768;
  }

  // There shouldn't be another defined range for the sensor
  //else{}

  // Debug purposes, print internal multiplier
  Serial.println(_internalMultiplier);
}

/**************************************************************************/
/*!
    @brief  Read the whoAmI register, and return its contents
*/
/**************************************************************************/
uint8_t Adafruit_ICM20789_ACC_Unified::getWhoAmI(void) {
  // Check device ID register
  return readRegisterICM20789(_i2c, MPUREG_WHO_AM_I);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X acc axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_ACC_Unified::getX(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t x=0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_L);
  x = aux2 | aux1<<8;
  return x;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y acc axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_ACC_Unified::getY(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t y=0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_L);
  y = aux2 | aux1<<8;
  return y;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z acc axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_ACC_Unified::getZ(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t z;
  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_L);
  z = aux2 | aux1<<8;
  return z;
}

bool Adafruit_ICM20789_ACC_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  // internal multiplier is obtained everytime a new range is set
  event->acceleration.x = getX() * _internalMultiplier;
  event->acceleration.y = getY() * _internalMultiplier;
  event->acceleration.z = getZ() * _internalMultiplier;

  return true;
}

void Adafruit_ICM20789_ACC_Unified::getSensor(sensor_t *sensor) {
    /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ICM20789_ACC", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 156.9064F; /* 16g = 156.9064 m/s^2  */
  sensor->min_value   = -156.9064F;  /*  -16g = -156.9064 m/s^2  */
  sensor->resolution  = 0.0006;   /*   0.0006 m/s^2 */

}

/**************************************************************************/
/*!
    @brief  Function used to get the xyz acc values from the sensor, and print
    its values using the serial port (testing function)
*/
/**************************************************************************/
void Adafruit_ICM20789_ACC_Unified::update_print_xyz() {
  // Update internal offset value, registers X,Y,and Z, after updating the
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t x,y,z =0;
  float px, py, pz = 0.0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_XOUT_L);
  x = aux2 | aux1<<8;
  px = x*_internalMultiplier;
  //px = (x * SENSORS_GRAVITY_STANDARD * 2)/32768;

  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_YOUT_L);
  y = aux2 | aux1<<8;
  py = y*_internalMultiplier;
  //py = (y * SENSORS_GRAVITY_STANDARD * 2)/32768;

  aux1 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_ACCEL_ZOUT_L);
  z = aux2 | aux1<<8;
  pz = z*_internalMultiplier;
  //pz = (z * SENSORS_GRAVITY_STANDARD * 2)/32768;

  Serial.print(px); Serial.print("  ");
  Serial.print(py); Serial.print("  ");
  Serial.print(pz); Serial.print("  ");
  //Serial.println();
}

/**************************************************************************/
/**************************************************************************/
/****************** GYROSCOPE functions and library ***********************/
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new ICM20789_GYRO class
*/
/**************************************************************************/
Adafruit_ICM20789_GYRO_Unified::Adafruit_ICM20789_GYRO_Unified(int32_t sensorID, uint8_t mode, range_gyro_icm_t rangeConfig) {
  _sensorID = sensorID;
  _i2c = mode;
  _range = rangeConfig;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ICM20789_GYRO_Unified::begin() {

  // This start is in configureICM20789
  if (_i2c)
    Wire.begin();
  else {
    SPI.begin();
  }

  // Set accel scale
  setRange(_range);
  return true;
}

/**************************************************************************/
/*!
    @brief  gets the current range value (from the class, not by reading)
*/
/**************************************************************************/
range_gyro_icm_t Adafruit_ICM20789_GYRO_Unified::getRange() {
  return _range;
}

/**************************************************************************/
/*!
    @brief  Sets a new range value
*/
/**************************************************************************/
void Adafruit_ICM20789_GYRO_Unified::setRange(range_gyro_icm_t range) {
  writeRegisterICM20789(_i2c, MPUREG_GYRO_CONFIG, (_range<<3) );

  // Based on the selected range, set a different internal multiplier
  if(range==ICM20789_GYRO_RANGE_250_DPS){
    _internalMultiplier =  (SENSORS_DPS_TO_RADS * 250)/32768;
  }

  else if(range==ICM20789_GYRO_RANGE_500_DPS){
    _internalMultiplier = (SENSORS_DPS_TO_RADS * 500)/32768;
  }

  else if(range==ICM20789_GYRO_RANGE_1000_DPS){
    _internalMultiplier = (SENSORS_DPS_TO_RADS * 1000)/32768;
  }

  else if(range==ICM20789_GYRO_RANGE_2000_DPS){
    _internalMultiplier = (SENSORS_DPS_TO_RADS * 2000)/32768;
  }

  // There shouldn't be another defined range for the sensor
  //else{}

  // Debug purposes, print internal multiplier
  Serial.println(_internalMultiplier);
}

/**************************************************************************/
/*!
    @brief  Read the whoAmI register, and return its contents
*/
/**************************************************************************/
uint8_t Adafruit_ICM20789_GYRO_Unified::getWhoAmI(void) {
  // Check device ID register
  return readRegisterICM20789(_i2c, MPUREG_WHO_AM_I);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X gyro axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_GYRO_Unified::getX(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t x=0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_XOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_XOUT_L);
  x = aux2 | aux1<<8;
  return x;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y gyro axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_GYRO_Unified::getY(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t y=0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_YOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_YOUT_L);
  y = aux2 | aux1<<8;
  return y;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z gyro axis value
*/
/**************************************************************************/
int16_t Adafruit_ICM20789_GYRO_Unified::getZ(void){
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t z;
  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_ZOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_ZOUT_L);
  z = aux2 | aux1<<8;
  return z;
}

bool Adafruit_ICM20789_GYRO_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = 0;
  // internal multiplier is obtained everytime a new range is set
  event->gyro.x = getX() * _internalMultiplier;
  event->gyro.y = getY() * _internalMultiplier;
  event->gyro.z = getZ() * _internalMultiplier;

  return true;
}

void Adafruit_ICM20789_GYRO_Unified::getSensor(sensor_t *sensor) {
    /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ICM20789_GYRO", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = 34.906586F; /* 2000DPS = 34.906586 rad/s  */
  sensor->min_value   = -34.906586F;
  sensor->resolution  = 0.000133;   /* rad/s @ 250DPS*/

}

/**************************************************************************/
/*!
    @brief  Function used to get the xyz acc values from the sensor, and print
    its values using the serial port (testing function)
*/
/**************************************************************************/
void Adafruit_ICM20789_GYRO_Unified::update_print_xyz() {
  // Update internal offset value, registers X,Y,and Z, after updating the
  uint8_t aux1, aux2 = 0; // aux1 will hold H byte, and aux2 will hold L byte
  int16_t x,y,z =0;
  float px, py, pz = 0.0;
  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_XOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_XOUT_L);
  x = aux2 | aux1<<8;
  px = x*_internalMultiplier;

  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_YOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_YOUT_L);
  y = aux2 | aux1<<8;
  py = y*_internalMultiplier;

  aux1 = readRegisterICM20789(_i2c, MPUREG_GYRO_ZOUT_H);
  aux2 = readRegisterICM20789(_i2c, MPUREG_GYRO_ZOUT_L);
  z = aux2 | aux1<<8;
  pz = z*_internalMultiplier;

  Serial.print(px); Serial.print("  ");
  Serial.print(py); Serial.print("  ");
  Serial.print(pz); Serial.print("  ");
  //Serial.println();
}

/**************************************************************************/
/**************************************************************************/
/******************* PRESSURE functions and library ***********************/
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new ICM20789_BARO class
*/
/**************************************************************************/
Adafruit_ICM20789_BARO_Unified::Adafruit_ICM20789_BARO_Unified(int32_t sensorID, uint8_t mode, uint8_t registersMode) {
  _sensorID = sensorID;
  _icmMode = registersMode;
  _i2cBypass = mode;

}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ICM20789_BARO_Unified::begin() {
  int16_t otp[4];
  int8_t out[3];


  // This start is in configureICM20789
  Wire.begin();

  // Based on i2c bypass selection, activate the bypass mode
  if(_i2cBypass){
    //writeRegisterICM20789(_icmMode, MPUREG_USER_CTRL, 0x02);
    writeRegisterICM20789(_icmMode, MPUREG_INT_PIN_CFG, 0x02);
  }

  // Perform a soft reset of the temp/baro sensor
  Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
  Wire.write(0x80);
  Wire.write(0x5D);
  Wire.endTransmission();
  delay(10);

  // Get the required coefficient parameters and store them in memory
  Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
  Wire.write(0xC5);
  Wire.write(0x95);
  Wire.write(0x00);
  Wire.write(0x66);
  Wire.write(0x9C);
  Wire.endTransmission();

  // Read OTP values
	for (int i = 0; i < 4; i++) {
		Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
    Wire.write(0xC7);
    Wire.write(0xF7);
    Wire.endTransmission();

    Wire.requestFrom(ICM20789_I2C_ADDRESS_PRESSURE, 3);
    if (Wire.available()) {
      out[0] = Wire.read();
      out[1] = Wire.read();
      out[2] = Wire.read();
    }
    else{
      Serial.print("Error requesting data");
    }
		otp[i] = out[0]<<8 | out[1];
	}

  _c1 = otp[0];
  _c2 = otp[1];
  _c3 = otp[2];
  _c4 = otp[3];

  return true;
}

/**************************************************************************/
/*!
    @brief  Read the device ID of the barometric sensor (same as temp)
*/
/**************************************************************************/
uint16_t Adafruit_ICM20789_BARO_Unified::getDeviceID(void) {
  uint16_t ID = 0;
  uint8_t out[3];

  // Check device ID register
  Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
  Wire.write(0xEF);
  Wire.write(0xC8);
  Wire.endTransmission();

  Wire.requestFrom(ICM20789_I2C_ADDRESS_PRESSURE, 3);
  if (Wire.available()) {
    out[0] = Wire.read();
    out[1] = Wire.read();
    out[2] = Wire.read();
  }
  else{
    Serial.print("Error requesting data");
  }
  ID = out[0]<<8 | out[1];

  return ID;
}


/**************************************************************************/
/*!
    @brief  Updates the raw data from the sensor (both temp and pressure)
*/
/**************************************************************************/
void Adafruit_ICM20789_BARO_Unified::updateRawData(void) {
  uint8_t rawData[9];

  // Send a measurement command (Trasmit P first, normal mode)
  Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
  Wire.write(0x48);
  Wire.write(0xA3);

  Wire.endTransmission();

  // There is a maximum delay of 6.3 mseconds after requesting new data
  delayMicroseconds(6500);
  //delay(100);

  // Get the pressure and temperature raw data
  Wire.beginTransmission(ICM20789_I2C_ADDRESS_PRESSURE);
  Wire.write(0xC5);
  Wire.write(0x95);
  Wire.write(0x00);
  Wire.write(0x66);
  Wire.write(0x9C);

  Wire.requestFrom(ICM20789_I2C_ADDRESS_PRESSURE, 9);

  int i =0;
  while(Wire.available()){    // slave may send less than requested
    char c = Wire.read();    // receive a byte as character
    //Serial.print(c, HEX);         // print the character
    rawData[i] = c;
    i++;
  }

  Wire.endTransmission();
  // Get the pressure and temperature raw data
  /*
  Serial.print("Data: ");
  Serial.print(rawData[0], HEX); Serial.print(" ");
  Serial.print(rawData[1], HEX); Serial.print(" ");
  Serial.print(rawData[2], HEX); Serial.print(" ");
  Serial.print(rawData[3], HEX); Serial.print(" ");
  Serial.print(rawData[4], HEX); Serial.print(" ");
  Serial.print(rawData[5], HEX); Serial.print(" ");
  Serial.print(rawData[6], HEX); Serial.print(" ");
  Serial.print(rawData[7], HEX); Serial.print(" ");
  Serial.print(rawData[8], HEX); Serial.println(" ");
  */
  temp_raw = rawData[6] << 8 | rawData[7];
	// Pressure raw data
	press_raw = (rawData[0]<<(8*2)) | (rawData[1]<<(8*1)) | (rawData[3]<<(8*0));

  // With the raw data, get the temperature data (this is done in another function)
  // LastTemperature = -45.f + 175.f/65536.f * (float)temp_raw;
}

/**************************************************************************/
/*!
    @brief  Updates the conversion constants, based on the raw temp read
    from the sensor
*/
/**************************************************************************/
void Adafruit_ICM20789_BARO_Unified::calculateConversionConstants(void) {
  float p_LUT[3];
  float t;
  // Those are all internal variables(or constants) of the class
  t = (float)(temp_raw - 32768);
  p_LUT[0] = LUT_lower + (float)(_c1 * t * t) * quadr_factor;
	p_LUT[1] = offst_factor * _c4 + (float)(_c2 * t * t) * quadr_factor;
	p_LUT[2] = LUT_upper + (float)(_c3 * t * t) * quadr_factor;

  C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
  p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
  p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
  (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
  p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
  p_LUT[1] * (p_Pa[2] - p_Pa[0]));

  A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) /
  (p_LUT[0] - p_LUT[1]);

  B = (p_Pa[0] - A) * (p_LUT[0] + C);
}

/**************************************************************************/
/*!
    @brief  Converts the raw data + internl coefficients to temp and pressure
*/
/**************************************************************************/
void Adafruit_ICM20789_BARO_Unified::processRawData(void) {
  LastPressure = A + B / (C + press_raw);
  // Pressure is obtained in pascals, convert it to HectoPascals
  LastPressure = LastPressure/100.0f;

  LastTemperature = -45.f + (175.f/65536.f) * temp_raw;
}

float Adafruit_ICM20789_BARO_Unified::getPress(void) {
  updateRawData();
  calculateConversionConstants();
  LastPressure = A + B / (C + press_raw);
  // Pressure is obtained in pascals, convert it to HectoPascals
  LastPressure = LastPressure/100.0f;

  return LastPressure;
}

float Adafruit_ICM20789_BARO_Unified::getTemp(void) {
  updateRawData();
  calculateConversionConstants();
  LastTemperature = -45.f + (175.f/65536.f) * temp_raw;

  return LastTemperature;
}

/**************************************************************************/
/*!
    @brief  Updates the internal variables pressure and temperature
*/
/**************************************************************************/
void Adafruit_ICM20789_BARO_Unified::getPressureAndTemperature(void) {
  updateRawData();
  calculateConversionConstants();
  processRawData();
}


bool Adafruit_ICM20789_BARO_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_PRESSURE;
  event->timestamp = 0;
  // Update internal values on call
  getPressureAndTemperature();
  event->pressure = LastPressure;

  return true;
}

void Adafruit_ICM20789_BARO_Unified::getSensor(sensor_t *sensor) {
    /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ICM20789_BARO", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0.01; //10ms
  sensor->max_value   = 1100; // 110kPa (Datasheet, normal range)
  sensor->min_value   = 700; //70kPa
  sensor->resolution  = 0.0001;   //0.01Pa
}

void Adafruit_ICM20789_BARO_Unified::print_pressure_temp(){
  Serial.print(LastPressure); Serial.print("  ");
  Serial.print(LastTemperature); Serial.print("  ");
}
