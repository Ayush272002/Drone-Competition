#include <Arduino.h>
#include "CommunicationsRB.h"

void correctDriftFromVectors(quaternion &current, vector estimate, vector measured, float timestep)
{
  vector ErrorAxis;
  ErrorAxis = estimate & measured; // find the axis around which one vector can be rotated to equal the direction of the other

  // measured.print("ERROR");

  if (ErrorAxis.magnitudeNoSqr() > 0)
  {

    ErrorAxis = normalize(ErrorAxis);                                     // make the above a unit vector
    ErrorAxis = ErrorAxis * -0.3 * estimate.vectorAngleBetween(measured); // spin the estimated attitude to converge these values. Spin faster if the value of acceleration looks close to 9.81ms2. If the value is far from this, it suggests that the body is accelerating and this is interferring with the acc
    ErrorAxis = rotateVector(ErrorAxis, current);

    if (!isnan(ErrorAxis.x) & !isnan(ErrorAxis.y) & !isnan(ErrorAxis.z))
    {
      current = applyW(current, ErrorAxis, timestep);
      // Serial.println("hi");
    }
  }
}