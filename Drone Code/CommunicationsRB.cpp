#include <Arduino.h>
#include "CommunicationsRB.h"
#include "quaternionRB.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

Receiver *Receiver::thisReceiver = 0;

Receiver::Receiver()
{
  thisReceiver = this;
}

bool Receiver::setupComms(uint8_t remoteAddress[6])
{
  WiFi.mode(WIFI_MODE_STA);
  // esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);
  Serial.println("Mac Address:" + WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESPNOW failed");
    // return false;
  }

  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_register_recv_cb(onDataRec);

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Connection failed, please reset");
    // return false;
  }
  return true;
}

void Receiver::onDataRec(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (thisReceiver != 0)
  {
    thisReceiver->getTransmittedData(mac, incomingData, len);
  }
}

void Receiver::getTransmittedData(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  memcpy(&transmitterValues, incomingData, sizeof(transmitterValues));

  target.theta = 1;
  target.vector[0] = 0;
  target.vector[1] = 0;
  target.vector[2] = 0;

  // TEST:
  targetYaw = (transmitterValues.yaw - 50) * PI / 8;
  target = eulerToQ(-(transmitterValues.roll - 50) * maxTiltAngle / 50.0, (transmitterValues.pitch - 50) * maxTiltAngle / 50.0, 0);

  if (abs(transmitterValues.throttle - 50) < 5)
  {
    transmitterValues.throttle = 50;
  }

  // Serial.println(String(target.theta,3) + "," + String(target.vector[0],3) + "," + String(target.vector[1],3) + "," + String(target.vector[2],3) + "," + "targetQuat" + ";");

  lastValidSignal = millis();
}
