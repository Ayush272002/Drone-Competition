#include <Arduino.h>
#include "IMURB.h"
#include "QuaternionRB.h"
#include <SPI.h>

// Definitions
#define NCS_MPU9250 18
#define degrees_per_second_2000_to_rads 0.00106526443 // convertion rate to radians/second: 2000*pi/(180*2^15)
#define degrees_per_second_1000_to_rads 0.00053263221
#define accelerometer_4g_to_meters_per_second2 0.00119750976 // convertion rate to m/s^2: 4*9.81/(2^15)

SPIClass *IMUHSpi = NULL;

volatile int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
volatile int interruptTriggerCount;
volatile float accXms2, accYms2, accZms2, gyroXrads, gyroYrads, gyroZrads;
byte imuDataRawAccGyro[14];
float offsetX, offsetY, offsetZ;
quaternion mountingAngle;

IMU::IMU(uint8_t _bus, float mountW, float mountX, float mountY, float mountZ)
{
  IMUHSpi = new SPIClass(_bus); // Clk is 14, MISO is 12 and MOSI is 13. On the GY-91, SDA is MOSI while SAO is MISO. VSpi is default, however I want to keep the pins on that side for future use
  mountingAngle.theta = mountW;
  mountingAngle.vector[0] = mountX;
  mountingAngle.vector[1] = mountY;
  mountingAngle.vector[2] = mountZ;
}

void IMU::init()
{

  pinMode(NCS_MPU9250, OUTPUT);
  digitalWrite(NCS_MPU9250, HIGH);
  // IMUHSpi->begin(14,12,26,27);  //Setup SPI, the MPU9250 can tolerate 2MHz for all registers
  IMUHSpi->begin(17, 15, 16, 47);
  IMUHSpi->setFrequency(1000000);
  IMUHSpi->setDataMode(SPI_MODE0);
  IMUHSpi->setBitOrder(MSBFIRST);

  writeIMUSPI(27, 0b00010000); // the two middle bits define the range of the gyro. Set them to the biggest range, since this makes the drone fly bigger
  writeIMUSPI(28, 0b00001000); // More or less same as above, however range is set to 4g since fighter jet level acceleration is unlikely (and if it happens then there's probably bigger issues)
  delay(10);
  if (readIMURegister(117) != 0x70)
  {
    Serial.println("IMU not found, takeoff may result in an unpleasant day");
    Serial.print("WHO AM I register returned:\t0x");
    Serial.println(readIMURegister(117), HEX);
    while (1)
    {
      ;
    }
  }
  else
  {
    Serial.println("IMU found");
  }

  varianceTotal = 100;
  while (varianceTotal > 4)
  {
    calibrate();
    delay(200);
  }

  Serial.write("IMU calibration complete");
}

void IMU::calibrate()
{
  int count = 0, sample = 400;
  IMUHSpi->setFrequency(20000000); // The main data registers can tolerate up to 20MHz clock rate
  varianceX = 0;
  varianceY = 0;
  varianceZ = 0;
  while (count < sample)
  {
    readIMUSPI(67, 6, imuDataRawAccGyro);
    gyroX = imuDataRawAccGyro[0] << 8 | imuDataRawAccGyro[1];
    gyroY = imuDataRawAccGyro[2] << 8 | imuDataRawAccGyro[3];
    gyroZ = imuDataRawAccGyro[4] << 8 | imuDataRawAccGyro[5];
    offsetX += gyroX;
    offsetY += gyroY;
    offsetZ += gyroZ;
    count++;
    varianceX += gyroX - (offsetX / count);
    varianceY += gyroY - (offsetY / count);
    varianceZ += gyroZ - (offsetZ / count);

    delay(1);
  }

  varianceX /= sample;
  varianceY /= sample;
  varianceZ /= sample;

  offsetX /= sample;
  offsetY /= sample;
  offsetZ /= sample;

  Serial.print("Calibration constants: ");
  Serial.print("X: ");
  Serial.print(offsetX);
  Serial.print("Y: ");
  Serial.print(offsetY);
  Serial.print("Z: ");
  Serial.println(offsetZ);

  Serial.print("Calibration Variance:\t");
  varianceTotal = sqrt(varianceX * varianceX + varianceY * varianceY + varianceZ * varianceZ);
  Serial.println(varianceTotal);

  IMUHSpi->setFrequency(1000000);
}

void IMU::handleImu()
{

  IMUHSpi->setFrequency(20000000); // The main data registers can tolerate up to 20MHz clock rate

  readIMUSPI(59, 14, imuDataRawAccGyro); // Read 14 registers: 2*3 for Acc, 2*1 for temp, 2*3 for gyro

  IMUHSpi->setFrequency(1000000); // The other registers can tolerate up to 2MHz clock rate, I revert to this to avoid glitches caused by the interrupt changing to 20MHz during read cycles

  accX = imuDataRawAccGyro[0] << 8 | imuDataRawAccGyro[1]; // Load data into int16 variables. This is in two's complement, so a 16 bit signed integer is neccessary to read negative values. Each axis has a high byte and a low byte, I shift the high byte by 8 bits and add the low byte
  accY = imuDataRawAccGyro[2] << 8 | imuDataRawAccGyro[3];
  accZ = imuDataRawAccGyro[4] << 8 | imuDataRawAccGyro[5];
  temperature = imuDataRawAccGyro[6] << 8 | imuDataRawAccGyro[7]; // I didn't want to read this, but it's quicker to just read it all in order than in two blocks. Digital suffering. Perhaps I'll use it to calibrate the barometer or something to save face.
  gyroX = imuDataRawAccGyro[8] << 8 | imuDataRawAccGyro[9];
  gyroY = imuDataRawAccGyro[10] << 8 | imuDataRawAccGyro[11];
  gyroZ = imuDataRawAccGyro[12] << 8 | imuDataRawAccGyro[13];

  IMUgyroRads.x = (gyroX * degrees_per_second_1000_to_rads - offsetX * degrees_per_second_1000_to_rads); // change units from arbitrary to meters per second squared and radians per second
  IMUgyroRads.y = (gyroY * degrees_per_second_1000_to_rads - offsetY * degrees_per_second_1000_to_rads);
  IMUgyroRads.z = (gyroZ * degrees_per_second_1000_to_rads - offsetZ * degrees_per_second_1000_to_rads);

  IMUaccms2.x = (accX * accelerometer_4g_to_meters_per_second2) - 0.2;
  IMUaccms2.y = (accY * accelerometer_4g_to_meters_per_second2) + 0;
  IMUaccms2.z = (accZ * accelerometer_4g_to_meters_per_second2) + 1;

  gyroRads = rotateVector(IMUgyroRads, mountingAngle);
  accms2 = rotateVector(IMUaccms2, mountingAngle);
}

// SPI functions
void IMU::writeIMUSPI(int reg, int value)
{
  digitalWrite(NCS_MPU9250, LOW); // Pull CS line for the MPU9250 low, beginning the transmittion

  IMUHSpi->transfer(reg); // Send register address

  IMUHSpi->transfer(value); // Send new value

  digitalWrite(NCS_MPU9250, HIGH); // End the transmittion by pulling the line high again

  delayMicroseconds(1); // delay required to prevent overlapping commands
}

void IMU::readIMUSPI(int reg, int count, byte data[])
{
  digitalWrite(NCS_MPU9250, LOW); // Pull CS line for the MPU9250 low, beginning the transmittion

  IMUHSpi->transfer(reg + 0x80); // Send the address for the first register to read, with the read bit set high (hence the + 0x80)

  for (int i = 0; i < count; i++)
  {
    data[i] = IMUHSpi->transfer(0x00); // Send a zero packet to request response for each register to read, load each into the byte array
  }

  digitalWrite(NCS_MPU9250, HIGH); // End the transmittion by pulling the line high again
  IMUHSpi->endTransaction();
}

int IMU::readIMURegister(int reg)
{
  int regData; // Effectively the same as readSPI, except it only reads one register and returns it for ease of use

  digitalWrite(NCS_MPU9250, LOW);

  IMUHSpi->transfer(reg + 0x80);
  regData = IMUHSpi->transfer(0x00);

  digitalWrite(NCS_MPU9250, HIGH);

  return regData;
}
