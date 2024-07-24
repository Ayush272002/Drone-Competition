#ifndef MotorRB_h
#define MotorRB_h
#include <Arduino.h>
#include "QuaternionRB.h"
// #include "ControlSettings.h"

class motor
{
private:
  vector torque;
  float thrustOut;
  float adjustment;
  int myChannel;

public:
  void setup(float torqueX, float torqueY, float torqueZ, int pinIn);
  int thrustToDuty(float thrust);
  void run(class vector aIn, float thrustIn);
  void printThrust();
  void setThrust(float thrust);
};

#endif