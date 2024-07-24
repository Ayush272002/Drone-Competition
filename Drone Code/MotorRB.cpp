#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include "MotorRB.h"
#include "QuaternionRB.h"

int freePWM = 1;
int escRate = 400;

void motor::setup(float torqueX, float torqueY, float torqueZ, int pinIn)
{
  torque.x = torqueX;
  torque.y = torqueY;
  torque.z = torqueZ;
  ledcSetup(freePWM, escRate, 14); // 65536 possible values
  Serial.println("Motor connected to PWM" + String(freePWM));
  ledcAttachPin(pinIn, freePWM); // TODO: fix this
  myChannel = freePWM;
  freePWM++;
}

int motor::thrustToDuty(float thrust)
{
  int duty;
  float maxDuty = int(16384 * (0.002 * escRate));
  float minDuty = int(16384 * (0.001 * escRate));
  duty = int(minDuty + (maxDuty - minDuty) * (thrust / 100));
  return duty;
}

void motor::run(class vector aIn, float thrustIn)
{

  thrustOut = thrustIn * 80;

  if (thrustOut > 80)
  {
    thrustOut = 80;
  }
  else if (thrustOut < 20)
  {
    thrustOut = 20;
  }

  adjustment = aIn * torque;

  if (adjustment > 20)
  {
    adjustment = 20;
  }
  else if (adjustment < -20)
  {
    adjustment = -20;
  }

  thrustOut += adjustment;

  setThrust(thrustOut);
}

void motor::printThrust()
{
  Serial.print(thrustOut);
  Serial.print(",");
}

void motor::setThrust(float thrust)
{
  thrustOut = thrust;
  ledcWrite(myChannel, thrustToDuty(thrust));
}