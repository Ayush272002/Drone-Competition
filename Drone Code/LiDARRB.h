#include <Wire.h>

// This code is based on the VL53Lx ULD, ported to work with the ESP32

const int address = 0x29;
int wireErrorVL53;

static const uint8_t status_rtn[24] = {255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
                                       255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                                       255, 255, 11, 12};

#define SOFT_RESET 0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS 0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define MM_CONFIG__INNER_OFFSET_MM 0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 0x0022
#define GPIO_HV_MUX__CTRL 0x0030
#define GPIO__TIO_HV_STATUS 0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP 0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI 0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A 0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B 0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI 0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO 0x0062
#define RANGE_CONFIG__SIGMA_THRESH 0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH 0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD 0x006C
#define SYSTEM__THRESH_HIGH 0x0072
#define SYSTEM__THRESH_LOW 0x0074
#define SD_CONFIG__WOI_SD0 0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0 0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE 0x0080
#define SYSTEM__SEQUENCE_CONFIG 0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 0x0082
#define SYSTEM__INTERRUPT_CLEAR 0x0086
#define SYSTEM__MODE_START 0x0087
#define VL53L1_RESULT__RANGE_STATUS 0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD 0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL 0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS 0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID 0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

void writeReg(uint16_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)(reg >> 8)); // reg high byte
  Wire.write((uint8_t)(reg));      // reg low byte
  Wire.write(value);
  wireErrorVL53 = Wire.endTransmission();
}

void writeReg16(uint16_t reg, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)(reg >> 8));   // reg high byte
  Wire.write((uint8_t)(reg));        // reg low byte
  Wire.write((uint8_t)(value >> 8)); // value high byte
  Wire.write((uint8_t)(value));      // value low byte
  wireErrorVL53 = Wire.endTransmission();
}

void writeReg32(uint16_t reg, uint16_t value)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)(reg >> 8)); // reg high byte
  Wire.write((uint8_t)(reg));      // reg low byte
  Wire.write((uint8_t)(value >> 24));
  Wire.write((uint8_t)(value >> 16));
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value));
  wireErrorVL53 = Wire.endTransmission();
}

uint8_t readReg(uint16_t reg)
{
  uint8_t val;

  Wire.beginTransmission(address);
  Wire.write((uint8_t)(reg >> 8)); // reg high byte
  Wire.write((uint8_t)(reg));      // reg low byte
  wireErrorVL53 = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  val = Wire.read();

  return val;
}

uint16_t readReg16(uint16_t reg)
{
  uint16_t val;

  Wire.beginTransmission(address);
  Wire.write((uint8_t)(reg >> 8)); // reg high byte
  Wire.write((uint8_t)(reg));      // reg low byte
  wireErrorVL53 = Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  val = (uint16_t)Wire.read() << 8; // value high byte
  val |= Wire.read();               // value low byte

  return val;
}

void VL53L1X_SetInterMeasurementInMs(uint32_t InterMeasMs)
{
  uint16_t Clock;

  Clock = readReg16(VL53L1_RESULT__OSC_CALIBRATE_VAL);
  Clock = Clock & 0x3FF;
  writeReg32(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
             (uint32_t)(Clock * InterMeasMs * 1.075));
}

int getRange()
{
  uint16_t distance;

  distance = readReg16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0);
  return distance;
}

/*
Status meanings:

0: winner winner 940nm dinner
1: "sigma failure" yep that's what the datasheet says. Honestly means the sensor is just whining for no reason.
2: "Signal failure" as bad as it sounds
4: "Out of bounds" data may be valid, but too far out to confirm. For our purposes, this is a 2
7: "Wraparound" shouldn't ever really happen.

Might make an enum if I have time, might not
*/

uint8_t getRangeStatus()
{
  uint8_t RgSt, rangeStatus;

  rangeStatus = 255;
  RgSt = readReg(VL53L1_RESULT__RANGE_STATUS);
  RgSt = RgSt & 0x1F;
  if (RgSt < 24)
    rangeStatus = status_rtn[RgSt];
  return rangeStatus;
}

void setToFastMode()
{
  writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
  writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
  VL53L1X_SetInterMeasurementInMs(20);
}

void startRanging()
{
  writeReg(SYSTEM__MODE_START, 0x40); // LiDAR time :D
}

void stopRanging()
{
  writeReg(SYSTEM__MODE_START, 0x00); // LiDAR become no :(
}

void clearInterrupt()
{ // Turn off the interrupt GPIO pin, which may or may not be used. Either way, I connected it on the FC
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
}

int getIntPolarity()
{
  int value;
  value = readReg(GPIO_HV_MUX__CTRL);
  value &= 0x10;
  return !(value >> 4);
}

boolean dataReadyHW()
{
  return digitalRead(12);
}

boolean dataReady()
{
  int intPolarity;
  int intStatus;
  intPolarity = getIntPolarity();
  intStatus = readReg(GPIO__TIO_HV_STATUS);

  if ((intStatus & 1) == intPolarity)
    return 1;
  else
    return 0;
}

int bootState()
{
  uint8_t booted;

  booted = readReg(VL53L1_FIRMWARE__SYSTEM_STATUS);

  Serial.println(booted);

  return booted & 0x01;
}

int initLiDAR()
{

  pinMode(12, INPUT);
  Wire.begin(13, 14);
  Wire.setClock(1000000);

  bool booted = false;
  while (!booted)
  {
    booted = bootState();
    delay(10);
  }
  Serial.println("LiDAR booted");

  uint8_t reg = 0x00, tmp;

  for (reg = 0x2D; reg <= 0x87; reg++)
  {
    writeReg(reg, VL51L1X_DEFAULT_CONFIGURATION[reg - 0x2D]);
  }
  startRanging();
  tmp = 0;
  while (tmp == 0)
  {
    tmp = dataReady();
  }
  clearInterrupt();
  stopRanging();
  writeReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
  writeReg(0x0B, 0);                                            /* start VHV from the previous temperature */

  delay(100);

  startRanging();

  return 0;
}
