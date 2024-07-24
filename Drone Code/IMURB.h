#ifndef IMURB_h
#define IMURB_h
#include <Arduino.h>
#include "QuaternionRB.h"
// #include <SPI.h>

class IMU
{
public:
  IMU(uint8_t _bus, float mountW, float mountX, float mountY, float mountZ);
  void init();
  void handleImu();
  vector accms2, gyroRads;
  // int NCS_MPU9250;//= 15;
  // float degrees_per_second_2000_to_rads;// = 0.00106526443; //convertion rate to radians/second: 2000*pi/(180*2^15)
  // float accelerometer_4g_to_meters_per_second2;// = 0.00119750976; //convertion rate to m/s^2: 4*9.81/(2^15)
private:
  void calibrate();
  quaternion mountingAngle;
  // hw_timer_t * readImuTimer = NULL; //Declare timer, but don't initialize it yet

  int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
  float varianceX, varianceY, varianceZ, varianceTotal;
  float offsetX, offsetY, offsetZ;
  byte imuDataRawAccGyro[14];
  vector IMUaccms2, IMUgyroRads;
  void writeIMUSPI(int reg, int value);
  void readIMUSPI(int reg, int count, byte data[]);
  int readIMURegister(int reg);
};
#endif