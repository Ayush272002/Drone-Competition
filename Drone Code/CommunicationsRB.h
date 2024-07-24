#ifndef CommunicationsRB_h
#define CommunicationsRB_h

#include <Arduino.h>
#include "quaternionRB.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const float maxTiltAngle = PI / 10;

typedef struct targetCommands_t
{
  float roll;
  float pitch;
  float yaw;
  float throttle;
  uint8_t estop;
} targetCommands_t;

class Receiver
{
private:
  bool foundTransmitter = false;
  static Receiver *thisReceiver;
  esp_now_peer_info_t peerInfo;
  static void onDataRec(const uint8_t *mac, const uint8_t *incomingData, int len);

public:
  targetCommands_t transmitterValues;
  quaternion target;
  Receiver();
  bool setupComms(uint8_t remoteAddress[6]);
  float targetYaw;
  int lastValidSignal;
  void getTransmittedData(const uint8_t *mac, const uint8_t *incomingData, int len);
};

#endif