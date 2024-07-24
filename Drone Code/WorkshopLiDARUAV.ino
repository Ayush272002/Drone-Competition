#include "QuaternionRB.h"
#include "ControlSettings.h"
#include "CommunicationsRB.h"
#include "SensorFusionRB.h"
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include "IMURB.h"
#include "MotorRB.h"
#include "LiDARRB.h"
#include <driver/adc.h>

int printLoops, lidarStatus, lastGoodAltitudeTime;
float lastGyroX, lastGyroZ, adjustedX, adjustedZ, opFlowSmoothX, opFlowSmoothZ, opFlowSumX, opFlowSumZ;

#define BENCH_TEST_MODE
// #define LIVE_TEST_MODE

IMU gy91(FSPI, 0.707, -0.02, 0, 0.707);

/* Axis definition looking at the drone from above and behind

     _____    X
    |     |   ^
    |     |   Z > Y
    |     |
    | ░░░ |
    |_____|

    Ye olde motor hub: white means ccw, black is cw

  Motor Wires:
  fl 38
  fr 36
  bl 39
  br 37

  battery monitor: GPIO1

  MAC: F4:12:FA:68:E3:AC
*/

typedef struct debugInfo_t
{
  char data[200];
} debugInfo_t;

debugInfo_t dataOut;

uint32_t startTime, endTime;
TaskHandle_t Task1;

// Vectors, quaternions and floats for the pose estimator
quaternion error, attitude; // the three pose quaternions: the desired, current and difference between the afforementioned
// quaternion IMUmountingAngle; //for eliminating any issues caused by an IMU that has been, purely for example, soldered into a peice of perfboard at an angle (and if this was something that happened, it was only a prototype anyway)
quaternion q; // used as a stand in for a static instance
vector estimatedGrav, measuredGrav, accErrorAxis, errorAxis, errorIntegral, PIDcorrection, errorProportional, errorDifferential;
vector down, gravity;

float errorMagnitude, halfSinErrorMagnitude;
float motorThrust;
vector angularVelocityVectorBody;
vector angularVelocityVector;
volatile float Xrads = 0, Yrads = 0, Zrads = 0;
volatile float batteryVoltage;
float colourSweep;
const int brightness = 127;

vector errorCheck, errorCheck2;
quaternion errorQuat;

unsigned long loopTime, lastPing, duration, lastTime, flagDuration;

int lidarInterval;
float lidarVelocity;
int underVoltageConsecutiveErrors, softUnderVoltageConsecutiveErrors;

// Objects/Structures for motor/data control
motor fr;
motor fl;
motor br;
motor bl;
Receiver receiver;

// void IRAM_ATTR updateIMU(){

//}

void setup()
{
  Serial.begin(115200);

  // pinMode(1, INPUT);

  Serial.print("Battery: ");
  batteryVoltage = 1 + (1.0 / (10.0 / (10.0 + 40.0))) * 3.3 * float(adc1_get_raw(ADC1_CHANNEL_0)) / 4096.0;
  // batteryVoltage=analogRead(1);
  Serial.println(batteryVoltage);

  receiver.setupComms(remoteAdd);

  fl.setup(1, 1, -1, 38);
  fr.setup(-1, 1, 1, 36);
  bl.setup(1, -1, 1, 39);
  br.setup(-1, -1, -1, 37);

  fr.setThrust(0);
  fl.setThrust(0);
  br.setThrust(0);
  bl.setThrust(0);

  down.x = 0;
  down.y = 0;
  down.z = -1;

  gravity.x = 0; // where we would expect to see gravitational acceleration of we assume the estimate is correct in the world frame
  gravity.y = 0;
  gravity.z = -9.81;

  pCon = pVal;
  iCon = iVal;
  dCon = dVal;
  iLim = iCap;

#ifndef BENCH_TEST_MODE
  while (receiver.transmitterValues.yaw < 30)
  {
    Serial.print(receiver.transmitterValues.yaw);
    Serial.println(" Waiting To Arm");
    neopixelWrite(48, (brightness + brightness * sin(colourSweep)), brightness + brightness * sin(colourSweep + 2 * PI / 3), brightness + brightness * sin(colourSweep + 4 * PI / 3));
    colourSweep += 0.1;
    delay(15);
  }
  while (receiver.transmitterValues.yaw > 10)
  {
    Serial.print(receiver.transmitterValues.yaw);
    Serial.println(" Waiting To Arm");
    neopixelWrite(48, (brightness + brightness * sin(colourSweep)), brightness + brightness * sin(colourSweep + 2 * PI / 3), brightness + brightness * sin(colourSweep + 4 * PI / 3));
    colourSweep += 0.1;
    delay(15);
  }
  while (receiver.transmitterValues.yaw < 30)
  {
    Serial.print(receiver.transmitterValues.yaw);
    Serial.println(" Waiting To Arm");
    neopixelWrite(48, (brightness + brightness * sin(colourSweep)), brightness + brightness * sin(colourSweep + 2 * PI / 3), brightness + brightness * sin(colourSweep + 4 * PI / 3));
    colourSweep += 0.1;
    delay(15);
  }
  neopixelWrite(48, brightness, 0, 0);
#endif

  timestep = 0.0005; // set time between interrupts, in seconds

  Serial.println("armed");

  gy91.init();
  initLiDAR();

  setToFastMode();
  // while (1) {;}
  delay(100);

  errorIntegral.x = 0;
  errorIntegral.y = 0;
  errorIntegral.z = 0;

  attitude.theta = 1;
  attitude.vector[0] = 0;
  attitude.vector[1] = 0;
  attitude.vector[2] = 0;

  armed = true;

  xTaskCreatePinnedToCore(printDumpster, "printDumpster", 10000, NULL, 0, &Task1, printCore);
}

void controlLoop()
{
  gy91.handleImu(); // get values from the IMU

  // Gyrometer stuff
  angularVelocityVectorBody = (gy91.gyroRads * 0.8 + angularVelocityVectorBody * 0.2); // Set this vector to the measurements from the gyroscope. These describe the speed of rotation about each axis, as viewed from the body frame since the IMU is glued/bolted (delete as neccessary) to it
  angularVelocityVector = angularVelocityVectorBody;                                   // copy the gyro values to another vector for the world frame
  angularVelocityVector = rotateVector(angularVelocityVector, attitude);               // transform the angular velocity to the world frame, using the current best pose estimate. On boot this is the identity quaternion
  attitude = applyW(attitude, angularVelocityVector, timestep);                        // integrate the angular velocity quaternion and store it in the pose estimate

  // Acceloscope stuff
  estimatedGrav = rotateVector(gravity, conj(attitude));
  measuredGrav = gy91.accms2 * 0.01 + measuredGrav * 0.99; //*0.6+measuredGrav*0.4; //create a vector from the accelerometer readings
  if (antiKick)
  {
    correctDriftFromVectors(attitude, estimatedGrav, measuredGrav, timestep);
  }
  else
  {
    antiKick = true;
  }

  /*
    accErrorAxis = estimatedGrav & measuredGrav;//find the axis around which one vector can be rotated to equal the direction of the other

    if (accErrorAxis.magnitudeNoSqr()>0){

      accErrorAxis = normalize(accErrorAxis);//make the above a unit vector

      accErrorAxis = accErrorAxis * -0.6 * estimatedGrav.vectorAngleBetween(measuredGrav);//spin the estimated attitude to converge these values. Spin faster if the value of acceleration looks close to 9.81ms2. If the value is far from this, it suggests that the body is accelerating and this is interferring with the acc

      accErrorAxis = rotateVector(accErrorAxis, attitude);

      if (!isnan(accErrorAxis.x) & !isnan(accErrorAxis.y) & !isnan(accErrorAxis.z)&antiKick){
        attitude = applyW(attitude, accErrorAxis,timestep);
        //attitude = q.applyW(attitude, down,timestep);
      }
      antiKick = true;
    }
  */

  /*
  currentYaw = -2*atan2(attitude.theta, attitude.vector[2]) + PI; //TODO: read "what is a quaternion and why is it better than an euler angle"
  yawCorrector.theta = cos(currentYaw/2);
  yawCorrector.vector[2] = sin(currentYaw/2);
  */

  yawCorrector.theta = attitude.theta;
  yawCorrector.vector[2] = attitude.vector[2];

  yawCorrector = qNormalize(yawCorrector);

  error = receiver.target * conj(conj(yawCorrector) * attitude); // compose the target pose with the opposite rotation of the current position to find the difference between them

  errorMagnitude = 2 * acos(error.theta); // a quaternion is effectively an axis angle measurement wrapped in trig. Unwrap this to convert to axis angle

  errorAxis = error.vector;

  halfSinErrorMagnitude = sin(errorMagnitude / 2);
  if (halfSinErrorMagnitude == 0)
  {                  // there is a potential NaN error coming up if magnitude is 2Pi or 0, aka if the drone is upside down or in the correct pose. This is because in both cases the best direction to travel in is undefined.
    errorAxis.x = 1; // in both cases, set the axis of rotation to be pitch. The exact direction doesn't matter as long as the drone turns somewhere if upside down, but pitching up is in many ways more intuitive than rolling. If in the correct pose, the magnitude is zero anyway
    errorAxis.y = 0;
    errorAxis.z = 0;
  }
  else
  {
    errorAxis = errorAxis * (1 / halfSinErrorMagnitude); // Divide away, free from the fear of NaN
  }

  errorAxis = errorAxis * errorMagnitude; // create a suitable angular velocity vector from the magnitude and a unit vector

  if (!(isnan(errorAxis.x) || isnan(errorAxis.y) || isnan(errorAxis.z)))
  { // double check that nothing is NaN :)
    // if (receiver.transmitterValues.throttle > 5){
    errorIntegral = errorIntegral + (errorAxis.elementwiseMultiplication(iCon)); // integrate away
    //}
  }

  errorProportional = errorAxis.elementwiseMultiplication(pCon);

  errorDifferential.x = (angularVelocityVectorBody.x * dCon.x) * 0.9 + 0.1 * errorDifferential.x; // TODO: dumbass
  errorDifferential.y = (angularVelocityVectorBody.y * dCon.y) * 0.9 + 0.1 * errorDifferential.y;
  errorDifferential.z = (angularVelocityVectorBody.z * dCon.z) * 0.9 + 0.1 * errorDifferential.z;

  errorDifferential.z -= receiver.targetYaw;

  // fr fl br bl

  // CHECK: addition of yaw velocity integral
  if (armed)
  {
    errorIntegral.x = constrain(errorIntegral.x, -iLim.x, iLim.x);
    errorIntegral.y = constrain(errorIntegral.y, -iLim.y, iLim.y);

    // errorIntegral.z += (-angularVelocityVectorBody.z + receiver.targetYaw)*yawAntiDrift;
    errorIntegral.z += (-errorDifferential.z) * yawAntiDrift;
    errorIntegral.z = constrain(errorIntegral.z, -iLim.z, iLim.z);
  }

  // now the whole team is here! errorAxis is a proportional Angular Velocity Vector (AVV), errorIntegral is the integral AVV correction, and AVVBody is the differential control signal
  PIDcorrection = errorIntegral + errorProportional - errorDifferential;

  // figure out how far the quad is from level, for thrust/lidar correction or really any sensor
  angleToLevel = down.vectorAngleBetween(rotateVector(down, attitude));

  targetHeight += (round(receiver.transmitterValues.throttle - 50)) * heightSensitivity;

  if (softUnderVoltageConsecutiveErrors > maxSoftUnderVoltageConsecutiveErrors)
  {
    targetHeight -= softEstopDropSpeed * heightSensitivity;
  }

  targetHeight = constrain(targetHeight, 0, altitudeLimit);

  altP = (targetHeight - altitude) * pAltVal;
  altI = constrain(altI + altP * iAltVal, -iAltCap, iAltCap);
  altD = ((lidarVelocity)*dAltVal) * 0.1 + 0.9 * altD;

  if (millis() - lastGoodAltitudeTime > millisBeforeLidarStop)
  {
    motorThrust = (55 / constrain(cos(angleToLevel), 0.8, 1)) / 100;
  }
  else
  {
    motorThrust = ((constrain((55 + altP + altI - altD), 40, 70)) / constrain(cos(angleToLevel), 0.8, 1)) / 100;
  }

  if (armed)
  {
#ifndef BENCH_TEST_MODE
    if (targetHeight < 25 & altitude < 55)
    {
      fl.run(PIDcorrection, 0.4);
      bl.run(PIDcorrection, 0.4);
      br.run(PIDcorrection, 0.4);
      fr.run(PIDcorrection, 0.4);
    }
    else
    {
      fl.run(PIDcorrection, motorThrust);
      bl.run(PIDcorrection, motorThrust);
      br.run(PIDcorrection, motorThrust);
      fr.run(PIDcorrection, motorThrust);
    }
#endif
  }
  else
  {
    fl.setThrust(0);
    bl.setThrust(0);
    br.setThrust(0);
    fr.setThrust(0);
  }

  if (((millis() - receiver.lastValidSignal) > 1500) || angleToLevel > crashDetectionAngle || receiver.transmitterValues.estop || (underVoltageConsecutiveErrors > maxUnderVoltageConsecutiveErrors))
  {
    armed = false;

    fl.setThrust(0);
    bl.setThrust(0);
    br.setThrust(0);
    fr.setThrust(0);

    neopixelWrite(48, 0, 255, 0);

    // Serial.println(batteryVoltage);
  }
  flagDuration = micros() - loopTime;
}

void printDumpster(void *parameter)
{
  while (1)
  {

    if (dataReady())
    {

      lidarRange = getRange();
      lidarStatus = getRangeStatus();

      if (lidarStatus == 0 || lidarStatus == 1)
      {
        lidarInterval = millis() - lastGoodAltitudeTime;
        oldAltitude = altitude;
        altitude = (lidarRange * constrain(cos(angleToLevel), 0.05, 1)) * lidarLPF + oldAltitude * (1 - lidarLPF);
        lidarVelocity = (altitude - oldAltitude) / lidarInterval;
        lastGoodAltitudeTime = millis();
        // Serial.println(altitude);
      }

      clearInterrupt();
    }
    batteryVoltage = 1 + (1.0 / (10.0 / (10.0 + 40.0))) * 3.3 * float(adc1_get_raw(ADC1_CHANNEL_0)) / 4096.0;

    if (batteryVoltage < voltageSoftCutoff)
    { // Is the voltage low?
      softUnderVoltageConsecutiveErrors++;
    }
    else
    {
      softUnderVoltageConsecutiveErrors--;
      if (softUnderVoltageConsecutiveErrors <= 0)
      {
        softUnderVoltageConsecutiveErrors = 0;
      }
    }

    if (batteryVoltage < voltageHardCutoff)
    { // Is the voltage really low? And no I'm not gonna nest this with the previous, it'd make my code ugly
      underVoltageConsecutiveErrors++;
    }
    else
    {
      underVoltageConsecutiveErrors--;
      if (underVoltageConsecutiveErrors <= 0)
      {
        underVoltageConsecutiveErrors = 0;
      }
    }

    if (millis() - lastPing > 50)
    {
      lastPing = millis();
      // q.printQuat(attitude, "Attitude");

      // Serial.print(motorThrust);
      // Serial.print(" ");
      // Serial.print(targetHeight);
      // Serial.print(" ");

      // Serial.print(batteryVoltage);

      /*
      Serial.print(targetHeight);
      Serial.print(" ");
      Serial.print(altitude);
      Serial.print(" ");
      Serial.print(lidarStatus);
      Serial.print(" ");
      Serial.print(lidarRange);
      Serial.print(" ");
      Serial.print(motorThrust);
      */

      /*
      Serial.print(gy91.accXms2);
      Serial.print("\t");
      Serial.print(gy91.accYms2);
      Serial.print("\t");
      Serial.print(gy91.accZms2);
      Serial.print("\t");
      */

      /*
        Serial.print("gyro:");
        Serial.print(periodAVelX,8);
        Serial.print(",");

        Serial.print("opflow:");
        Serial.print((-(float(opFlowSmoothX)*(PI))*1.48),8);
        Serial.print(",");

        Serial.print("adj:");
        Serial.print(-periodAVelX+(-(float(opFlowSmoothX)*(PI))*1.45),8);
        Serial.print(",");
      */

      //}

      // Serial.print(",");
      // Serial.print("adj:");
      // Serial.print(((-(float(pmw.xVelRads)*(PI/180))*2.5)-lastGyroX)*1+adjustedX*0);

      // Serial.print(adjustedX,8);

      /*
      Serial.print("X:");
      Serial.print(adjustedX);
      Serial.print(",");
      Serial.print("Z:");
      Serial.print(adjustedZ);
      Serial.print(",");
      */

      Serial.print("FL:");
      fl.printThrust();
      Serial.print("FR:");
      fr.printThrust();
      Serial.print("BL:");
      bl.printThrust();
      Serial.print("BR:");
      br.printThrust();

      // Serial.print(loopTime-lastTime);
      // Serial.print("\t");
      // Serial.print(flagDuration);
      // Serial.print("\t");
      // Serial.print(lidarVelocity);
      // Serial.print("\t");
      // Serial.print(batteryVoltage);
      // Serial.print("\t");
      // Serial.print(underVoltageConsecutiveErrors);

      // Serial.print(tfMini.range);

      // Serial.print("\t");
      // Serial.print(String(attitude.theta,5) + "," + String(attitude.vector[0],5) + "," + String(attitude.vector[1],5) + "," + String(attitude.vector[2],5) + "," + "attitude" + ";");

      // Serial.print(String(error.theta,5) + "," + String(error.vector[0],5) + "," + String(error.vector[1],5) + "," + String(error.vector[2],5) + "," + "errorQuat" + ";");
      // Serial.print(String(receiver.target.theta,5) + "," + String(receiver.target.vector[0],5) + "," + String(receiver.target.vector[1],5) + "," + String(receiver.target.vector[2],5) + "," + "targetQuat" + ";");
      // Serial.print(String(errorCheck.x,5) + "," +String(errorCheck.y,5) + "," +String(errorCheck.z,5) + "," + "1" + ";");
      // Serial.print(String(accErrorAxis.x,5) + "," +String(accErrorAxis.y,5) + "," +String(accErrorAxis.z,5) + "," + "accErrorAxis" + ";");
      // Serial.print(String(estimatedGrav.x,5) + "," +String(estimatedGrav.y,5) + "," +String(estimatedGrav.z,5) + "," + "estimated" + ";");
      // Serial.print(String(measuredGrav.x,5) + "," +String(measuredGrav.y,5) + "," +String(measuredGrav.z,5) + "," + "measured" + ";");

      // Serial.print(String(errorProportional.x,5) + "," +String(errorProportional.y,5) + "," +String(errorProportional.z,5) + "," + "errorProp" + ";");
      // Serial.print(String(errorIntegral.x,5) + "," +String(errorIntegral.y,5) + "," +String(errorIntegral.z,5) + "," + "errorIntegral" + ";");
      // Serial.print(String(errorDifferential.x,5) + "," +String(errorDifferential.y,5) + "," +String(errorDifferential.z,5) + "," + "errorDiff" + ";");

      // errorCheck.print("error");

      // Serial.print(String(altP,5) + "," +String(altI,5) + "," +String(altD,5) + "," + "PID" + ";");

      // Serial.print(altitude);
      // Serial.print(",");
      // Serial.print(angleToLevelCos);

      // Serial.print(flagDuration);

      // fl.printThrust();
      // bl.printThrust();
      // br.printThrust();
      // fr.printThrust();
      // Serial.print(receiver.lastValidSignal);

      // gy91.gyroRads.print("Gyro");
      // gy91.accms2.print("Acc");
      // Serial.print(String(attitude.theta,5) + "," + String(attitude.vector[0],5) + "," + String(attitude.vector[1],5) + "," + String(attitude.vector[2],5) + "," + "attitude" + ";");
      Serial.println();

      String stringus = String(attitude.theta, 5) + "," + String(attitude.vector[0], 5) + "," + String(attitude.vector[1], 5) + "," + String(attitude.vector[2], 5) + "," + "attitude" + ";";
      stringus += String(estimatedGrav.x) + "," + String(estimatedGrav.y) + "," + String(estimatedGrav.z) + "," + "estimated" + ";";
      stringus += String(measuredGrav.x) + "," + String(measuredGrav.y) + "," + String(measuredGrav.z) + "," + "real" + ";";
      // stringus +=  + "Batt" + String(batteryVoltage);
      // stringus += "," + String(errorIntegral.y) + "," + String(errorProportional.y);
      stringus += String(receiver.target.theta, 5) + "," + String(receiver.target.vector[0], 5) + "," + String(receiver.target.vector[1], 5) + "," + String(receiver.target.vector[2], 5) + "," + "target" + ";";

      stringus.toCharArray(dataOut.data, stringus.length() + 1);
      // dataOut.data = "hi";
      // Serial.print(dataOut.data);
      // esp_err_t sentSuccessfully = esp_now_send(remoteAdd, (uint8_t *) &dataOut, sizeof(dataOut));
    }
  }
}

int counter;

void loop()
{

  attitude.theta = 1;
  attitude.vector[0] = 0;
  attitude.vector[1] = 0;
  attitude.vector[2] = 0;

  // attitude.printQuat(attitude, "att");

  while (1)
  {
    if ((micros() - loopTime) >= 500)
    {
      lastTime = loopTime;
      loopTime = micros();
      timestep = float(loopTime - lastTime) / 1000;
      controlLoop();

      duration = (micros() - loopTime);

      counter++;
    }
  }
}
