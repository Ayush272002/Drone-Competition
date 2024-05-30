#include <esp_now.h>
#include <WiFi.h>
#include "driver/adc.h"
#include <esp_wifi.h>

#define INVERT_ROLL
#define INVERT_PITCH
#define INVERT_YAW
#define INVERT_THROTTLE

//0C:B8:15:C1:2A:E0

//0xDC,0x54,0x75,0xED,0x70,0xD0
typedef struct debugInfo_t{
  char data[200];
} debugInfo_t;

debugInfo_t dataIn;
//uint8_t targetAdd[] = {0xDC,0x54,0x75,0xED,0x70,0xD4};
uint8_t targetAdd[] = {0xDC, 0x54, 0x75, 0xED, 0x70, 0xC0}; //mac address of the drone 

const int offsetZone = 0;
float maxR = 4092 - offsetZone, minR = 0 + offsetZone, maxP = 4092- offsetZone, minP = 0 + offsetZone, maxY = 4092- offsetZone, minY = 0 + offsetZone, maxT = 4092- offsetZone, minT = 0 + offsetZone;
float zeroR = 4, zeroP = 4, zeroY = 5, zeroT = 6;

typedef struct targetCommands{
  float rollStick;
  float pitchStick;
  float yawStick;
  float thrustStick;
  uint8_t estop;
} targetCommands;

targetCommands dataOut;
esp_now_peer_info_t peerInfo;

void setup() {
  pinMode(0,INPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);
  analogRead(32);

  esp_now_init();
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, targetAdd, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Connection failed, please reset");
    while(1);
  }
}

float rollSmooth, pitchSmooth, thrustSmooth, yawSmooth;
float rollProcessed, pitchProcessed, thrustProcessed, yawProcessed;
int i, lastSent;

const float stickLPF = 1;

void loop() {

  //pins
  float yaw    =   scale100(analogRead(34), minR, maxR, zeroR);
  float thrust =   scale100(analogRead(35), minP, maxP, zeroP);
  float roll   =   scale100(analogRead(32), minY, maxY, zeroY);
  float pitch  =   scale100(analogRead(33), minT, maxT, zeroT);

  rollSmooth   = rollSmooth   * (1-stickLPF) + roll   * stickLPF;
  pitchSmooth  = pitchSmooth  * (1-stickLPF) + pitch  * stickLPF;
  yawSmooth    = yawSmooth    * (1-stickLPF) + yaw    * stickLPF;
  thrustSmooth = thrustSmooth * (1-stickLPF) + thrust * stickLPF;

  //Serial.println(String(rollSmooth) + " " + String(pitchSmooth) + " " + String(yawSmooth) + " " + String(thrustSmooth));
  #ifdef INVERT_ROLL
  rollProcessed   = 100 - rollSmooth;
  #else
  rollProcessed   = rollSmooth;
  #endif

  #ifdef INVERT_PITCH
  pitchProcessed  = 100 - pitchSmooth;
  #else
  pitchProcessed  = pitchSmooth;
  #endif

  #ifdef INVERT_YAW
  yawProcessed    = 100 - yawSmooth;
  #else
  yawProcessed    = yawSmooth;
  #endif

  #ifdef INVERT_THROTTLE
  thrustProcessed = 100 - thrustSmooth;
  #else
  thrustProcessed = thrustSmooth;
  #endif
  

  dataOut.rollStick   = rollProcessed;
  dataOut.pitchStick  = pitchProcessed;
  dataOut.yawStick    = yawProcessed;
  dataOut.thrustStick = thrustProcessed;

  if (!digitalRead(0)){
    dataOut.estop = 1;
  }

  //Serial.println(String(thrust) + "\t" + String(roll) + "\t" + String(pitch) + "\t" + String(yaw));

  if(millis()-lastSent>50){
    lastSent = millis();
    esp_err_t sentSuccessfully = esp_now_send(targetAdd, (uint8_t *) &dataOut, sizeof(dataOut));
    //Serial.println(String(analogRead(39)) + " " + String(analogRead(36)) + " " + String(analogRead(34)) + " " + String(analogRead(35)));
    Serial.println(String(rollProcessed) + " " + String(pitchProcessed) + " " + String(yawProcessed) + " " + String(thrustProcessed) + " " + String(dataOut.estop));
  }
  i++;
  //delay(2);
}

float scale100(float val, float min, float max, float offset){
  return constrain(100 * (((val-min)/(max-min)))+offset, 0, 100);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  if (status != ESP_NOW_SEND_SUCCESS){
    
  }
}
