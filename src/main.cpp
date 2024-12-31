#include <ModbusSerial.h>

#include "modbus_ethernet.h"

// Modbus config
#define AZI_ROT_REG_1 0x22
#define AZI_ROT_REG_2 0x23
#define LED_PIN PB0

// Serial modbus config
#define BAUDRATE 38400
#define SLAVE_ID 0x01
#define REG 0x22

// MOTOR config
const int shifterEn = PB1; // A6
#define PUL_PIN PB12       // Pulse pin (PUL+)(D19)
#define DIR_PIN PB15       // Direction pin (DIR+)(D17)
#define STEPS_PER_REV 400  // DM542 pulse/rev

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // mac

HardwareSerial Serial1(PG9, PG14); // D0, D1 of stm32f767zi

ModbusEthernet mb_eth;

ModbusSerial mb_serial(Serial1, SLAVE_ID, -1);

void rotateToAngle(float);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial1.begin(BAUDRATE, SERIAL_8E1);
  while (!Serial1)
  {
    ;
  }

  mb_eth.config(mac);
  mb_serial.config(BAUDRATE);

  // 32 bit float registers 16 bit each
  mb_eth.addHreg(AZI_ROT_REG_1, 0x0000);
  mb_eth.addHreg(AZI_ROT_REG_2, 0x0000);

  mb_serial.addHreg(AZI_ROT_REG_1, 0x0000);
  mb_serial.addHreg(AZI_ROT_REG_2, 0x0000);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  pinMode(shifterEn, OUTPUT);
  digitalWrite(shifterEn, HIGH);
}

void loop()
{
  mb_eth.task();
  mb_serial.task();

  // TCP check

  uint16_t regVal = mb_eth.hreg(AZI_ROT_REG_1);
  uint16_t regVal2 = mb_eth.hreg(AZI_ROT_REG_2);

  uint32_t n = (regVal << 16) | regVal2;
  float *newRotationAngle = (float *)&n;

  if (*newRotationAngle != 0)
  {
    rotateToAngle(*newRotationAngle);
    mb_eth.setHreg(AZI_ROT_REG_1, 0x0);
    mb_eth.setHreg(AZI_ROT_REG_2, 0x0);
    Serial.print("Rotation angle(TCP): ");
    Serial.println(*newRotationAngle, 2);
  }

  // RTU check
  regVal = mb_serial.hreg(AZI_ROT_REG_1);
  regVal2 = mb_serial.hreg(AZI_ROT_REG_2);

  n = (regVal << 16) | regVal2;
  newRotationAngle = (float *)&n;

  if (*newRotationAngle != 0)
  {
    rotateToAngle(*newRotationAngle);
    mb_serial.setHreg(AZI_ROT_REG_1, 0x0);
    mb_serial.setHreg(AZI_ROT_REG_2, 0x0);
    Serial.print("Rotation angle(RTU): ");
    Serial.println(*newRotationAngle, 2);
  }

  delay(30);
}

// Function to rotate the motor to a specific angle
void rotateToAngle(float angle)
{
  int steps = (int)((angle / 360.0) * STEPS_PER_REV); // Calculate steps for the given angle

  if (steps < 0)
  {
    digitalWrite(DIR_PIN, LOW); // Set direction to counterclockwise
    steps = -steps;             // Convert to positive steps
  }
  else
  {
    digitalWrite(DIR_PIN, HIGH); // Set direction to clockwise
  }

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(PUL_PIN, HIGH); // Pulse ON
    delayMicroseconds(500);      // Adjust for speed
    digitalWrite(PUL_PIN, LOW);  // Pulse OFF
    delayMicroseconds(500);      // Adjust for speed
  }
}
