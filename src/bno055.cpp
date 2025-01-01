#include "bno055.h"

BNO055::BNO055()
{
  // Constructor
}

bool BNO055::begin()
{
  Wire.begin(BN_SDA, BN_SCL);
  delay(50); // Give the sensor some time to initialize

  // Read the Chip ID from the BNO055 to verify the connection
  uint8_t chipID = getChipID();
  Serial.print("chip id: ");
  Serial.println(chipID);

  if (chipID != 0xA0)
  {
    delay(1000); // hold on for boot
    // id = read8();
    chipID = getChipID();
    if (chipID != 0xA0)
    {
      return false; // still not? ok bail
    }
  }

  // Set the operation mode to NDOF
  writeRegister(BNO055_OPR_MODE, BNO055_MODE_NDOF);
  delay(20); // Delay for the mode switch

  return true;
}

uint8_t BNO055::getChipID()
{
  return read8(BNO055_CHIP_ID_REG); // Read the chip ID from the sensor
}

void BNO055::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BNO055_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
    delay(10);
}

void BNO055::getEulerAngles(float &heading, float &roll, float &pitch)
{
  uint8_t buffer[6];
  readLen(BNO055_EULER_H_L, buffer, 6);

  // Convert raw data to Euler angles
  heading = (float)((int16_t)(buffer[1] << 8 | buffer[0])) / 16.0;
  roll = (float)((int16_t)(buffer[3] << 8 | buffer[2])) / 16.0;
  pitch = (float)((int16_t)(buffer[5] << 8 | buffer[4])) / 16.0;
}

void BNO055::getMagnetometer(float &magX, float &magY, float &magZ)
{
  uint8_t buffer[6];
  readLen(BNO055_MAG_X_LSB, buffer, 6);

  // Convert raw data to magnetometer values (in µT)
  // magX = (float)((int16_t)(buffer[1] << 8 | buffer[0])) / 16.0;
  // magY = (float)((int16_t)(buffer[3] << 8 | buffer[2])) / 16.0;
  // magZ = (float)((int16_t)(buffer[5] << 8 | buffer[4])) / 16.0;
  magX = ((float)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8))) / 16.0;
  magY = ((float)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8))) / 16.0;
  magZ = ((float)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8))) / 16.0;
}

float BNO055::getMagneticNorth()
{
  float magX, magY, magZ;
  getMagnetometer(magX, magY, magZ);

  // Calculate magnetic north in degrees
  float heading = atan2(magY, magX) * 180.0 / M_PI; // Convert radians to degrees

  // Normalize to 0-360 degrees
  if (heading < 0)
  {
    heading += 360.0;
  }
  return heading;
}

void BNO055::getAccelerometer(float &accX, float &accY, float &accZ)
{
  uint8_t buffer[6];
  readLen(BNO055_ACC_X_LSB, buffer, 6);

  // Convert raw data to accelerometer values (in m/s^2)
  accX = (float)((int16_t)(buffer[1] << 8 | buffer[0])) / 100.0;
  accY = (float)((int16_t)(buffer[3] << 8 | buffer[2])) / 100.0;
  accZ = (float)((int16_t)(buffer[5] << 8 | buffer[4])) / 100.0;
}

void BNO055::getGyroscope(float &gyroX, float &gyroY, float &gyroZ)
{
  uint8_t buffer[6];
  readLen(BNO055_GYRO_X_LSB, buffer, 6);

  // Convert raw data to gyroscope values (in degrees per second)
  gyroX = (float)((int16_t)(buffer[1] << 8 | buffer[0])) / 900.0; // Adjust the scaling factor
  gyroY = (float)((int16_t)(buffer[3] << 8 | buffer[2])) / 900.0;
  gyroZ = (float)((int16_t)(buffer[5] << 8 | buffer[4])) / 900.0;
}

bool BNO055::isSensorReady()
{
  uint8_t status = read8(BNO055_SYS_STATUS);
  return (status == 0x01); // Check if the sensor is in normal operation mode
}

uint8_t BNO055::read8(uint8_t reg)
{
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDRESS, (uint8_t)1);
  return Wire.read();
}

void BNO055::readLen(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDRESS, len);
  for (uint8_t i = 0; i < len; i++)
  {
    buffer[i] = Wire.read();
  }
}
