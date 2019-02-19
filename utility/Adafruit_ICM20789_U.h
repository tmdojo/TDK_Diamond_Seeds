/**************************************************************************/
/*!
    @file     Adafruit_ICM20789.h
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

#include "utility/Adafruit_Sensor.h"
#include <Wire.h>
#include <SPI.h>

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define MPUREG_WHO_AM_I 0x75
    #define MPUREG_SMPLRT_DIV 0x19
    #define MPUREG_XG_OFFS_USRH 0x13
    #define MPUREG_YG_OFFS_USRH 0x15
    #define MPUREG_ZG_OFFS_USRH 0x17
    #define MPUREG_CONFIG 0x1a
    #define MPUREG_GYRO_CONFIG 0x1b
    #define MPUREG_ACCEL_CONFIG 0x1c
    #define MPUREG_ACCEL_CONFIG_2 0x1d
    #define MPUREG_LP_CONFIG 0x1e
    #define MPUREG_ACCEL_WOM_THR 0x1f
    #define MPUREG_ACCEL_WOM_X_THR 0x20
    #define MPUREG_ACCEL_WOM_Y_THR 0x21
    #define MPUREG_ACCEL_WOM_Z_THR 0x22
    #define MPUREG_FIFO_EN 0x23
    #define MPUREG_INT_PIN_CFG 0x37
    #define MPUREG_INT_ENABLE 0x38
    #define MPUREG_INT_STATUS 0x3a
    #define MPUREG_USER_CTRL 0x6a
    #define MPUREG_PWR_MGMT_1 0x6b
    #define MPUREG_PWR_MGMT_2 0x6c
    #define MPUREG_BANK_SEL 0x6d
    #define MPUREG_MEM_START_ADDR 0x6e
    #define MPUREG_MEM_R_W 0x6f
    #define MPUREG_FIFO_COUNTH 0x72
    #define MPUREG_FIFO_R_W 0x74
    #define MPUREG_XA_OFFS_H 0x77
    #define MPUREG_YA_OFFS_H 0x7A
    #define MPUREG_ZA_OFFS_H 0x7D

    #define MPUREG_ACCEL_XOUT_H 0x3B
    #define MPUREG_ACCEL_XOUT_L 0x3C
    #define MPUREG_ACCEL_YOUT_H 0x3D
    #define MPUREG_ACCEL_YOUT_L 0x3E
    #define MPUREG_ACCEL_ZOUT_H 0x3F
    #define MPUREG_ACCEL_ZOUT_L 0x40
    #define MPUREG_TEMP_XOUT_H 0x41
    #define MPUREG_TEMP_XOUT_L 0x42
    #define MPUREG_GYRO_XOUT_H 0x43
    #define MPUREG_GYRO_XOUT_L 0x44
    #define MPUREG_GYRO_YOUT_H 0x45
    #define MPUREG_GYRO_YOUT_L 0x46
    #define MPUREG_GYRO_ZOUT_H 0x47
    #define MPUREG_GYRO_ZOUT_L 0x48

    #define MPUREG_SELF_TEST_X_ACCEL 0x0D
    #define MPUREG_SELF_TEST_Y_ACCEL 0x0E
    #define MPUREG_SELF_TEST_Z_ACCEL 0x0F

    #define MPUREG_SIGNAL_PATH_RESET 0x68
    #define MPUREG_ACCEL_INTEL_CTRL  0x69
/*=========================================================================*/

// User custom definitions
#define USE_I2C 1
#define ICM20789_I2C_ADDRESS 0x68
#define ICM20789_I2C_ADDRESS_PRESSURE 0x63
#define SAMPLE_RATE_DIV 0 // Could be either number from 0-255, where samplerate=1khz/(1+div)
#define USE_I2C_PRESSURE_BYPASS 1 //If set, use the principal I2C connection, otherwise, use the auxiliar

// Main functions used for both the accelerometer and the gyroscope
uint8_t readRegisterICM20789(uint8_t mode, uint8_t reg);
void writeRegisterICM20789(uint8_t mode, uint8_t reg, uint8_t value);
void configureICM20789(uint8_t mode, uint8_t sample_rate=SAMPLE_RATE_DIV);
void startICM20789_6axis(uint8_t mode);
void stopICM20789_6axis(uint8_t mode);

// Class definitions
// Define possible range values for the accelerometer
// Used with register MPUREG_ACCEL_CONFIG
typedef enum
{
  ICM20789_ACC_RANGE_16_G          = 0b11,   // +/- 16g
  ICM20789_ACC_RANGE_8_G           = 0b10,   // +/- 8g
  ICM20789_ACC_RANGE_4_G           = 0b01,   // +/- 4g
  ICM20789_ACC_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_acc_icm_t;

class Adafruit_ICM20789_ACC_Unified : public Adafruit_Sensor {
 public:
  Adafruit_ICM20789_ACC_Unified(int32_t sensorID = -1, uint8_t mode = USE_I2C, range_acc_icm_t rangeConfig=ICM20789_ACC_RANGE_2_G);

  bool       begin(void);
  bool       getEvent(sensors_event_t*);
  void       getSensor(sensor_t*);

  void update_print_xyz();
  range_acc_icm_t getRange();
  void setRange(range_acc_icm_t range);
  uint8_t getWhoAmI();

  int16_t getX(void), getY(void), getZ(void);

  private:
  int32_t _sensorID;
  uint8_t _i2c;
  range_acc_icm_t _range;
  float _internalMultiplier;

};

// Class definitions
// Define possible range values for the gyroscope
// Used with register MPUREG_GYRO_CONFIG
typedef enum
{
  ICM20789_GYRO_RANGE_2000_DPS          = 0b11,   // +/- 2000DPS
  ICM20789_GYRO_RANGE_1000_DPS          = 0b10,   // +/- 1000DPS
  ICM20789_GYRO_RANGE_500_DPS           = 0b01,   // +/- 500DPS
  ICM20789_GYRO_RANGE_250_DPS           = 0b00    // +/- 250DPS (default value)
} range_gyro_icm_t;

class Adafruit_ICM20789_GYRO_Unified : public Adafruit_Sensor {
 public:
  Adafruit_ICM20789_GYRO_Unified(int32_t sensorID = -1, uint8_t mode = USE_I2C, range_gyro_icm_t rangeConfig=ICM20789_GYRO_RANGE_250_DPS);

  bool       begin(void);
  bool       getEvent(sensors_event_t*);
  void       getSensor(sensor_t*);

  void update_print_xyz();
  range_gyro_icm_t getRange();
  void setRange(range_gyro_icm_t range);
  uint8_t getWhoAmI();

  int16_t getX(void), getY(void), getZ(void);

  private:
  int32_t _sensorID;
  uint8_t _i2c;
  range_gyro_icm_t _range;
  float _internalMultiplier;
};


class Adafruit_ICM20789_BARO_Unified : public Adafruit_Sensor {
 public:
  Adafruit_ICM20789_BARO_Unified(int32_t sensorID = -1, uint8_t mode = USE_I2C_PRESSURE_BYPASS, uint8_t registersMode = USE_I2C);

  bool       begin(void);
  bool       getEvent(sensors_event_t*);
  void       getSensor(sensor_t*);

  uint16_t getDeviceID(void);

  void getPressureAndTemperature();

  float LastPressure;
  float LastTemperature;
  void print_pressure_temp();
  float getPress(void), getTemp(void);

  private:

  // Common adafruit sensor variables
  int32_t _sensorID;
  uint8_t _icmMode;
  uint8_t _i2cBypass;

  // Main functions used to process the raw pressure data
  void updateRawData();
  void calculateConversionConstants();
  void processRawData();

  // internal pressure coefficients (init at startup, reading the sensor)
  int16_t _c1, _c2, _c3, _c4;
  // internal constants used by the pressure algorithm
  const float p_Pa[3] = {45000.0, 80000.0, 105000.0};
	const float LUT_lower = 3.5 * (1<<20);
	const float LUT_upper = 11.5 * (1<<20);
	const float quadr_factor = 1 / 16777216.0;
	const float offst_factor = 2048.0;

  // internal temp and pressure data
  int32_t press_raw;
  int32_t temp_raw;

  // internal values used to calculate the pressure from raw values
  // these variables depend on the current temperature
  //float p_LUT[3];
  float A, B, C; // pressure = (A+B)/(C+press_raw)

};
