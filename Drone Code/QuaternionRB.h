#ifndef QuaternionRB_h
#define QuaternionRB_h
#include <Arduino.h>

class quaternion qNormalize(quaternion qIn);
class quaternion applyW(quaternion qIn, float vin[], float timestep);
class quaternion applyW(quaternion qIn, class vector vin, float timestep);
float qMagnitude(quaternion qIn);
void rotateVector(float vin[], quaternion qIn);
class vector rotateVector(class vector vin, quaternion qIn);
class quaternion conj(quaternion qIn);
class quaternion vectorToQ(float vin[]);
class quaternion vectorToQ(class vector vin);
class quaternion eulerToQ(float yaw, float pitch, float roll);
class quaternion HPNN(quaternion q1, quaternion q2);
void printQuat(quaternion q1, String tag);
class vector normalize(vector vin);

class quaternion
{
public:
  float theta = 1;
  float vector[3] = {0, 0, 0};
  quaternion operator*(quaternion q2);
  quaternion operator*(float postMult);
  quaternion operator+(quaternion qIn2);
  void assign(quaternion qNew);
  // quaternion functions
};

class vector
{
public:
  // vector();
  // vector(float x, float y, float z);
  float x = 0, y = 0, z = 0;
  vector operator*(float postMult);
  vector elementwiseMultiplication(vector vin);
  vector operator+(vector vIn);
  vector operator-(vector vIn);
  vector operator&(vector vin);
  float vectorAngleBetween(vector vin);
  float operator*(vector vIn);
  void operator=(float vin[3]);
  vector operator+(float vin[3]);
  vector operator+=(vector vin);
  float magnitude();
  float magnitudeNoSqr();
  void print(String tag);
};

#endif