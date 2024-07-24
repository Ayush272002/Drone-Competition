#include <Arduino.h>
#include "quaternionRB.h"

quaternion quaternion::operator*(quaternion q2)
{
  quaternion qOut;

  qOut.theta = theta * q2.theta - vector[0] * q2.vector[0] - vector[1] * q2.vector[1] - vector[2] * q2.vector[2];
  qOut.vector[0] = theta * q2.vector[0] + vector[0] * q2.theta + vector[1] * q2.vector[2] - vector[2] * q2.vector[1];
  qOut.vector[1] = theta * q2.vector[1] - vector[0] * q2.vector[2] + vector[1] * q2.theta + vector[2] * q2.vector[0];
  qOut.vector[2] = theta * q2.vector[2] + vector[0] * q2.vector[1] - vector[1] * q2.vector[0] + vector[2] * q2.theta;

  return qOut;
}

quaternion quaternion::operator*(float postMult)
{
  quaternion qOut;

  qOut.theta = theta * postMult;
  qOut.vector[0] = vector[0] * postMult;
  qOut.vector[1] = vector[1] * postMult;
  qOut.vector[2] = vector[2] * postMult;

  return qOut;
}

quaternion quaternion::operator+(quaternion qIn2)
{
  quaternion qOut;

  qOut.theta = theta + qIn2.theta;
  qOut.vector[0] = vector[0] + qIn2.vector[0];
  qOut.vector[1] = vector[1] + qIn2.vector[1];
  qOut.vector[2] = vector[2] + qIn2.vector[2];

  return qOut;
}

void quaternion::assign(quaternion qNew)
{
  theta = qNew.theta;
  vector[0] = qNew.vector[0];
  vector[1] = qNew.vector[1];
  vector[2] = qNew.vector[2];
}

// quaternion functions
quaternion qNormalize(quaternion qIn)
{
  quaternion qOut;
  float magnitude = 1 / qMagnitude(qIn);
  qOut.theta = qIn.theta * magnitude;
  qOut.vector[0] = qIn.vector[0] * magnitude;
  qOut.vector[1] = qIn.vector[1] * magnitude;
  qOut.vector[2] = qIn.vector[2] * magnitude;
  return qOut;
}

quaternion applyW(quaternion quatToRotate, float angularVelocity[], float timestep)
{
  timestep /= 1000;
  quaternion qOut;
  quaternion qW;
  qW = vectorToQ(angularVelocity);
  qOut = quatToRotate + (qW * (timestep * 0.5)) * quatToRotate;
  return qNormalize(qOut);
}

quaternion applyW(quaternion quatToRotate, class vector angularVelocity, float timestep)
{
  timestep /= 1000;
  quaternion qOut;
  quaternion qW;
  qW = vectorToQ(angularVelocity);
  qOut = quatToRotate + (qW * (timestep * 0.5)) * quatToRotate;
  return qNormalize(qOut);
}

float qMagnitude(quaternion qIn)
{
  float magnitude = (sqrt(qIn.theta * qIn.theta + qIn.vector[0] * qIn.vector[0] + qIn.vector[1] * qIn.vector[1] + qIn.vector[2] * qIn.vector[2]));
  return magnitude;
}

quaternion vectorToQ(float vin[])
{
  quaternion qOut;
  qOut.vector[0] = vin[0];
  qOut.vector[1] = vin[1];
  qOut.vector[2] = vin[2];
  qOut.theta = 0;
  return qOut;
}

quaternion vectorToQ(class vector vin)
{
  quaternion qOut;
  qOut.vector[0] = vin.x;
  qOut.vector[1] = vin.y;
  qOut.vector[2] = vin.z;
  qOut.theta = 0;
  return qOut;
}

quaternion eulerToQ(float roll, float pitch, float yaw)
{
  quaternion qOut;

  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5); // 0:x 1:y 2:z

  qOut.vector[0] = sr * cp * cy - cr * sp * sy;
  qOut.vector[1] = cr * sp * cy + sr * cp * sy;
  qOut.vector[2] = cr * cp * sy - sr * sp * cy;
  qOut.theta = cr * cp * cy + sr * sp * sy;

  return qOut;
}

void rotateVector(float vin[], quaternion qIn)
{
  float vOut[3];
  quaternion qV;
  qV = vectorToQ(vin);
  qV = (qIn * qV) * conj(qIn);
  vOut[0] = qV.vector[0];
  vOut[1] = qV.vector[1];
  vOut[2] = qV.vector[2];
  vin[0] = vOut[0];
  vin[1] = vOut[1];
  vin[2] = vOut[2];
}

class vector rotateVector(class vector vin, quaternion qIn)
{
  class vector vOut;
  quaternion qV;
  qV = vectorToQ(vin);
  qV = (qIn * qV) * conj(qIn);
  vOut.x = qV.vector[0];
  vOut.y = qV.vector[1];
  vOut.z = qV.vector[2];
  vin.x = vOut.x;
  vin.y = vOut.y;
  vin.z = vOut.z;
  return vOut;
}

quaternion HPNN(quaternion q1, quaternion q2)
{
  quaternion qOut;

  qOut.theta = q1.theta * q2.theta - q1.vector[0] * q2.vector[0] - q1.vector[1] * q2.vector[1] - q1.vector[2] * q2.vector[2];
  qOut.vector[0] = q1.theta * q2.vector[0] + q1.vector[0] * q2.theta + q1.vector[1] * q2.vector[2] - q1.vector[2] * q2.vector[1];
  qOut.vector[1] = q1.theta * q2.vector[1] - q1.vector[0] * q2.vector[2] + q1.vector[1] * q2.theta + q1.vector[2] * q2.vector[0];
  qOut.vector[2] = q1.theta * q2.vector[2] + q1.vector[0] * q2.vector[1] - q1.vector[1] * q2.vector[0] + q1.vector[2] * q2.theta;

  return qOut;
}

void printQuat(quaternion q1, String tag)
{
  Serial.print(String(q1.theta, 7) + "," + String(q1.vector[0], 7) + "," + String(q1.vector[1], 7) + "," + String(q1.vector[2], 7) + "," + tag + ";");
}

quaternion conj(quaternion qIn)
{
  quaternion qOut;
  qOut.theta = qIn.theta;
  qOut.vector[0] = -qIn.vector[0];
  qOut.vector[1] = -qIn.vector[1];
  qOut.vector[2] = -qIn.vector[2];
  return qOut;
}

/* TODO: readd this
  vector::vector(){
    x = 0;
    y = 0;
    z = 0;
  }

  vector::vector(float _x, float _y, float _z){
    x = _x;
    y = _y;
    z = _z;
  }
*/
vector vector::operator*(float postMult)
{
  vector vOut;
  vOut.x = x * postMult;
  vOut.y = y * postMult;
  vOut.z = z * postMult;

  return vOut;
}

vector vector::operator+(vector vIn)
{
  vector vOut;

  vOut.x = x + vIn.x;
  vOut.y = y + vIn.y;
  vOut.z = z + vIn.z;

  return vOut;
}

vector vector::operator-(vector vIn)
{
  vector vOut;

  vOut.x = x - vIn.x;
  vOut.y = y - vIn.y; // TODO: you moron, you utter muppet
  vOut.z = z - vIn.z;

  return vOut;
}

vector vector::operator&(vector vin)
{ //& contains a cross, therefore cross product
  vector vOut;

  vOut.x = y * vin.z - z * vin.y;
  vOut.y = z * vin.x - x * vin.z;
  vOut.z = x * vin.y - y * vin.x;

  return vOut;
}

vector normalize(vector vin)
{
  vector vOut;

  vOut = (vin * (1 / vin.magnitude()));

  return vOut;
}

float vector::vectorAngleBetween(vector vin)
{
  float sOut;

  sOut = acos((vin.x * x + vin.y * y + vin.z * z) / sqrt(magnitudeNoSqr() * vin.magnitudeNoSqr()));

  return sOut;
}

float vector::operator*(vector vIn)
{
  float sOut;

  sOut = x * vIn.x + y * vIn.y + z * vIn.z;

  return sOut;
}

void vector::operator=(float vin[3])
{
  x = vin[0];
  y = vin[1];
  z = vin[2];
}

vector vector::operator+(float vin[3])
{
  vector vOut;
  vOut.x = x + vin[0];
  vOut.y = y + vin[1];
  vOut.z = z + vin[2];
  return vOut;
}

vector vector::operator+=(vector vin)
{
  vector vOut;
  vOut.x = x + vin.x;
  vOut.y = y + vin.y;
  vOut.z = z + vin.z;
  return vOut;
}

vector vector::elementwiseMultiplication(vector vin)
{
  vector vout;

  vout.x = vin.x * x;
  vout.y = vin.y * y;
  vout.z = vin.z * z;

  return vout;
}

float vector::magnitudeNoSqr()
{
  return (x * x + y * y + z * z);
}

float vector::magnitude()
{
  return sqrt(x * x + y * y + z * z);
}

void vector::print(String tag)
{
  Serial.print(String(x, 12) + "," + String(y, 12) + "," + String(z, 12) + "," + tag + ";");
}
