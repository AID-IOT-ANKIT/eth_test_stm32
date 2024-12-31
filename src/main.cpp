#include "modbus_ethernet.h"

// Modbus config
#define HREG 100
#define LED_PIN PB0

// MOTOR config
const int shifterEn = PB1; // A6
#define PUL_PIN PB12       // Pulse pin (PUL+)(D19)
#define DIR_PIN PB15       // Direction pin (DIR+)(D17)
#define STEPS_PER_REV 400  // DM542 pulse/rev

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // mac

float rotationAngle = 0.0;

ModbusEthernet mb;

void rotateToAngle(float);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  mb.config(mac);

  mb.addHreg(HREG, 0x0000);
  mb.addHreg(HREG + 1, 0x0000);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  pinMode(shifterEn, OUTPUT);
  digitalWrite(shifterEn, HIGH);
}

void loop()
{
  mb.task();

  uint16_t regVal = mb.hreg(HREG);
  uint16_t regVal2 = mb.hreg(HREG + 1);

  uint32_t n = (regVal << 16) | regVal2;
  float *newRotationAngle = (float *)&n;

  if(*newRotationAngle != rotationAngle) {
    rotationAngle = *newRotationAngle;
    rotateToAngle(rotationAngle);
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
