#ifndef BNO055_H
#define BNO055_H

#include <Wire.h>

// BNO055 I2C Address
#define BNO055_ADDRESS 0x28 // I2C address of the BNO055 sensor

// Register definitions
#define BNO055_CHIP_ID_REG 0x00  // Chip ID register
#define BNO055_EULER_H_L 0x1A    // Euler angles (Heading, Roll, Pitch) registers
#define BNO055_ACC_X_LSB 0x08    // Accelerometer data (X-axis)
#define BNO055_GYRO_X_LSB 0x14   // Gyroscope data (X-axis)
#define BNO055_SYS_STATUS 0x39   // System status register
#define BNO055_MAG_X_LSB 0x0E    // Magnetometer data (X-axis)
#define BNO055_OPR_MODE 0x3D     // Operation mode register
#define BNO055_MODE_MAGONLY 0x02 // Magnetic-only mode
#define BNO055_MODE_NDOF 0x0C    // NDOF mode ()

class BNO055
{
    const uint32_t BN_SDA = PC9;
    const uint32_t BN_SCL = PA8;

public:
    BNO055();
    bool begin();                                                   // Initialize sensor
    uint8_t getChipID();                                            // Get the Chip ID
    void getEulerAngles(float &heading, float &roll, float &pitch); // Get Euler angles
    void getAccelerometer(float &accX, float &accY, float &accZ);   // Get accelerometer data
    void getGyroscope(float &gyroX, float &gyroY, float &gyroZ);    // Get gyroscope data
    void getMagnetometer(float &magX, float &magY, float &magZ);    // Get magnetometer data
    float getMagneticNorth();                                       // Calculate magnetic north from magnetometer
    bool isSensorReady();                                           // Check if the sensor is ready

private:
    uint8_t read8(uint8_t reg);                              // Read a single byte from a register
    void readLen(uint8_t reg, uint8_t *buffer, uint8_t len); // Read multiple bytes from a register
    void writeRegister(uint8_t reg, uint8_t value);
};

#endif
