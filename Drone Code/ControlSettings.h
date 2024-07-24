#ifndef ControlSettings_h
#define ControlSettings_h

#include <Arduino.h>
// Control variables. All of these should be alterable to change the behaviour of the UAV without unexpected concequences. Change other variables at your own risk.
float pVal[] = {45, 45, 45};  // 15;
float iVal[] = {0.1, 0.1, 0}; // 0.0004;
// float iVal[] = {0, 0, 0};//0.0004;
float dVal[] = {4, 4, 10};

float yawAntiDrift = 0.008;
// float yawAntiDrift = 0;

float iCap[] = {8, 8, 6}; // 1.7;

float pAltVal = 0.045;
float iAltVal = 0.000015;
float dAltVal = 15;
float iAltCap = 0.65;
float lidarLPF = 0.5;

const float voltageHardCutoff = 10;
const float voltageSoftCutoff = 10.5;
const int maxUnderVoltageConsecutiveErrors = 30;
const int maxSoftUnderVoltageConsecutiveErrors = 30;
const int softEstopDropSpeed = 25; // If this is negative, when the battery gets low the drone will try to escape to the moon
const float crashDetectionAngle = 1.51;
const int millisBeforeLidarStop = 2000;

const float heightSensitivity = 0.01;
const float altitudeLimit = 1300;
// 30:C6:F7:05:6D:60
uint8_t remoteAdd[] = {0x0C, 0xDC, 0x7E, 0xCA, 0xD6, 0x6C}; // controller address for telemetry 0C:DC:7E:CA:D6:6C

// Definition variables. Change these to experience manmade horrors beyond comprehension.
// #define NCS_MPU9250 27 //The gy-91 is hooked up to pin 15, subject to change.
// #define degrees_per_second_2000_to_rads 0.00106526443 //convertion rate to radians/second: 2000*pi/(180*2^15)
// #define accelerometer_4g_to_meters_per_second2 0.00119750976 //convertion rate to m/s^2: 4*9.81/(2^15)
float timestep = 0.0005;

bool armed = false;
bool antiKick = false;
float angleToLevel, angleToLevelCos;
float currentYaw;
float avgAVelX, avgAVelZ, avgACount;
float altitude, oldAltitude, velocity, lidarRange;
quaternion yawCorrector, testQuat;
vector pCon, iCon, dCon, iLim, measuredGravRaw;
float altP, altI, altD, altError, targetHeight;
float throttleCorrection;
uint16_t tempRange;

int printCore = 0;

#endif