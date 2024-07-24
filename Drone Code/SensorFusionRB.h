#ifndef SensorFusionRB_h
#define SensorFusionRB_h
#include <Arduino.h>
#include "CommunicationsRB.h"

void correctDriftFromVectors(quaternion &current, vector estimate, vector measured, float timestep);

#endif