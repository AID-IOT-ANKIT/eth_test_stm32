#include <ModbusSerial.h>

#include "modbus_ethernet.h"
#include "bno055.h"

// Encoder config
#define AZ_A PB11
#define AZ_B PB8

#define EL_A PC2
#define EL_B PA0

// Modbus config
#define AZI_ROT_REG_1 0x22
#define AZI_ROT_REG_2 0x23

#define ELE_ROT_REG_1 0x42
#define ELE_ROT_REG_2 0x43

#define MAG_REG_1 0x30
#define MAG_REG_2 0x31

#define LED_PIN PB0

// Serial modbus config
#define BAUDRATE 38400
#define SLAVE_ID 0x01
#define REG 0x22

// MOTOR config
const int shifterEn = PB1; // A6
const int shiftEn2 = PA15; // D20

// motor relay -> D28
#define MOTOR_RELAY PD13

#define PUL_AZ PB12 // Pulse pin (PUL+)(D19)
#define DIR_AZ PB15 // Direction pin (DIR+)(D17)

#define PUL_EL PB3
#define DIR_EL PB5

#define GEAR 50
#define PULSE 400
#define STEPS_PER_REV (PULSE * GEAR) // DM542 pulse/rev

#define SPEED 1000 // Desired speed in steps per second (can be adjusted)

const int AZ_M = PD15;
const int AZ_P = PA4;
const int EL_P = PB2;

const float P_GAIN = 1.35; // Proportional gain
const float I_GAIN = 0.05; // Integral gain
const float D_GAIN = 0.01; // Derivative gain

int zeroPosition = 0;           // Zero position reference
float error, prevError = 0;     // PID errors
float integral = 0, derivative; // PID components
float motorSpeed;

uint32_t sensor_save_time = 0;

HardwareSerial Serial1(PG9, PG14); // D0, D1 of stm32f767zi

ModbusEthernet mb_eth;

ModbusSerial mb_serial(Serial1, SLAVE_ID, -1);

BNO055 bno_sensor;

volatile long encoderCountAZ = 0;
volatile long encoderCountEL = 0;

// void rotateToAngleAZ(float);
// void rotateToAngleEL(float);
void rotateToAngle(float, uint16_t, uint16_t, bool);

void moveToAngle(float);

float currentPositionAZ();
float currentPositionEL();

void readEncoderAZ();
void readEncoderEL();

void az_home();
void el_home();

void saveMagValue();

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

  if (!bno_sensor.begin())
  {
    Serial.println("BNO055 Initialization Failed!");
  }

  mb_eth.config();
  mb_serial.config(BAUDRATE);

  // 32 bit float registers 16 bit each
  mb_eth.addHreg(AZI_ROT_REG_1, 0x0000);
  mb_eth.addHreg(AZI_ROT_REG_2, 0x0000);

  // elevation motor register
  mb_eth.addHreg(ELE_ROT_REG_1, 0x0);
  mb_eth.addHreg(ELE_ROT_REG_2, 0x0);

  mb_eth.addHreg(MAG_REG_1, 0x0);
  mb_eth.addHreg(MAG_REG_2, 0x0);

  mb_serial.addHreg(AZI_ROT_REG_1, 0x0000);
  mb_serial.addHreg(AZI_ROT_REG_2, 0x0000);

  pinMode(PUL_AZ, OUTPUT);
  pinMode(DIR_AZ, OUTPUT);

  pinMode(PUL_EL, OUTPUT);
  pinMode(DIR_EL, OUTPUT);

  pinMode(AZ_A, INPUT);
  pinMode(AZ_B, INPUT);

  pinMode(EL_A, INPUT);
  pinMode(EL_B, INPUT);

  pinMode(AZ_M, INPUT);
  pinMode(AZ_P, INPUT_PULLUP);
  pinMode(EL_P, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(AZ_B), readEncoderAZ, RISING);
  attachInterrupt(digitalPinToInterrupt(EL_B), readEncoderEL, RISING);

  pinMode(shifterEn, OUTPUT);
  pinMode(shiftEn2, OUTPUT);
  pinMode(MOTOR_RELAY, OUTPUT);
  digitalWrite(shifterEn, HIGH);
  digitalWrite(shiftEn2, HIGH);
  digitalWrite(MOTOR_RELAY, HIGH);

  // home both motors
  el_home();
  az_home();

  encoderCountAZ = 0;
  encoderCountEL = 0;
}

void loop()
{
  if(millis() - sensor_save_time > 500) {
    sensor_save_time = millis();
    saveMagValue();
  }
  mb_eth.task();

  // TCP check

  uint16_t regVal = mb_eth.hreg(AZI_ROT_REG_1);
  uint16_t regVal2 = mb_eth.hreg(AZI_ROT_REG_2);

  uint32_t n = (regVal << 16) | regVal2;
  float *newRotationAngle = (float *)&n;

  float angle = 0.0;

  if (*newRotationAngle != 0)
  {
    angle = *newRotationAngle;

    rotateToAngle(angle, DIR_AZ, PUL_AZ, true);
    // moveToAngle(angle);

    mb_eth.setHreg(AZI_ROT_REG_1, 0x0);
    mb_eth.setHreg(AZI_ROT_REG_2, 0x0);
    Serial.print("Rotation angle(AZ): ");
    Serial.println(*newRotationAngle, 2);
  }

  regVal = mb_eth.hreg(ELE_ROT_REG_1);
  regVal2 = mb_eth.hreg(ELE_ROT_REG_2);

  n = (regVal << 16) | regVal2;
  newRotationAngle = (float *)&n;

  if (*newRotationAngle != 0)
  {
    angle = *newRotationAngle;
    rotateToAngle(angle, DIR_EL, PUL_EL, false);
    mb_eth.setHreg(ELE_ROT_REG_1, 0x0);
    mb_eth.setHreg(ELE_ROT_REG_2, 0x0);
    Serial.print("Rotation angle(EL): ");
    Serial.println(*newRotationAngle, 2);
  }

  // mb_serial.task();

  // // RTU check
  // regVal = mb_serial.hreg(AZI_ROT_REG_1);
  // regVal2 = mb_serial.hreg(AZI_ROT_REG_2);

  // n = (regVal << 16) | regVal2;
  // newRotationAngle = (float *)&n;

  // if (*newRotationAngle != 0)
  // {
  //   angle = *newRotationAngle;
  //   mb_serial.setHreg(AZI_ROT_REG_1, 0x0);
  //   mb_serial.setHreg(AZI_ROT_REG_2, 0x0);
  //   Serial.print("Rotation angle(RTU): ");
  //   Serial.println(*newRotationAngle, 2);
  //   rotateToAngle(*newRotationAngle);
  // }

  delay(30);
}

void floatToRegisters(float value, uint16_t &regHigh, uint16_t &regLow)
{
  uint32_t floatBits = *((uint32_t *)&value); // Interpret float as 32-bit int
  regHigh = (floatBits >> 16) & 0xFFFF;       // Extract high 16 bits
  regLow = floatBits & 0xFFFF;                // Extract low 16 bits
}

void saveMagValue()
{
  uint16_t regHigh, regLow;

  float magnetic_north = bno_sensor.getMagneticNorth();

  floatToRegisters(magnetic_north, regHigh, regLow);

  mb_eth.setHreg(MAG_REG_1, regHigh);
  mb_eth.setHreg(MAG_REG_2, regLow);
}

// PID-based movement function
// void moveToAngle(float targetAngle)
// {
//   float currentAngle = 0;
//   float error = 0;
//   float previousError = 0;
//   float integral = 0;
//   unsigned long lastUpdateTime = 0;

//   while (true)
//   {
//     currentAngle = currentPosition();

//     // Compute PID
//     error = targetAngle - currentAngle;
//     integral += error;
//     float derivative = error - previousError;
//     float output = P_GAIN * error + I_GAIN * integral + D_GAIN * derivative;

//     previousError = error;

//     // Determine direction
//     if (output > 0)
//     {
//       digitalWrite(DIR_PIN, HIGH); // Clockwise
//     }
//     else
//     {
//       digitalWrite(DIR_PIN, LOW); // Counterclockwise
//       output = -output;           // Make output positive
//     }

//     // Move motor based on PID output
//     for (int i = 0; i < (int)output; i++)
//     {
//       digitalWrite(PUL_PIN, HIGH);
//       delayMicroseconds(500); // Adjust pulse width for speed
//       digitalWrite(PUL_PIN, LOW);
//       delayMicroseconds(500);
//     }

//     // Break condition: Stop when error is small enough
//     if (abs(error) < 0.1) // Allowable tolerance
//     {
//       break;
//     }

//     // Update PID loop at regular intervals
//     unsigned long now = millis();
//     if (now - lastUpdateTime < 10) // 10 ms loop interval
//     {
//       continue;
//     }
//     lastUpdateTime = now;
//   }
// }

float currentPositionAZ()
{
  noInterrupts();
  long count = encoderCountAZ; // Safely copy the count
  interrupts();

  // Calculate the position in degrees

  float position = (float)count / 3600.0 * 360.0;

  Serial.print("Position (AZ): ");
  Serial.println(position); // Print the position

  return position;
}

float currentPositionEL()
{
  noInterrupts();
  long count = encoderCountEL; // Safely copy the count
  interrupts();

  // Calculate the position in degrees

  float position = (float)count / 3600.0 * 360.0;

  Serial.print("Position (EL): ");
  Serial.println(position); // Print the position

  return position;
}

void az_home()
{
  digitalWrite(DIR_AZ, LOW); // clockwise

  while (digitalRead(AZ_P) == HIGH)
  {
    auto _time_per_step = 1.0 / SPEED * 1000000;
    digitalWrite(PUL_AZ, HIGH);            // Pulse ON
    delayMicroseconds(_time_per_step / 2); // Adjust for speed
    digitalWrite(PUL_AZ, LOW);             // Pulse OFF
    delayMicroseconds(_time_per_step / 2); // Adjust for speed
  }
}

void el_home()
{
  digitalWrite(DIR_EL, LOW); // clockwise

  while (digitalRead(EL_P) == HIGH)
  {
    auto _time_per_step = 1.0 / SPEED * 1000000;
    digitalWrite(PUL_EL, HIGH);            // Pulse ON
    delayMicroseconds(_time_per_step / 2); // Adjust for speed
    digitalWrite(PUL_EL, LOW);             // Pulse OFF
    delayMicroseconds(_time_per_step / 2); // Adjust for speed
  }
}

void readEncoderAZ()
{
  int a = digitalRead(AZ_A);
  a > 0 ? encoderCountAZ++ : encoderCountAZ--;
}

void readEncoderEL()
{
  int a = digitalRead(EL_A);
  a > 0 ? encoderCountEL++ : encoderCountEL--;
}

// // Function to rotate the motor to a specific angle
// void rotateToAngle(float angle)
// {
//   int steps = (int)((deltaAngle / 360.0) * STEPS_PER_REV); // Calculate steps for the given angle

//   if (steps < 0)
//   {
//     digitalWrite(DIR_PIN, LOW); // Set direction to counterclockwise
//     steps = -steps;             // Convert to positive steps
//   }
//   else
//   {
//     digitalWrite(DIR_PIN, HIGH); // Set direction to clockwise
//   }

//   auto _time_per_step = 1.0 / SPEED * 1000000;

//   for (int i = 0; i < steps; i++)
//   {
//     digitalWrite(PUL_PIN, HIGH);           // Pulse ON
//     delayMicroseconds(_time_per_step / 2); // Adjust for speed
//     digitalWrite(PUL_PIN, LOW);            // Pulse OFF
//     delayMicroseconds(_time_per_step / 2); // Adjust for speed
//   }
// }

void rotateToAngle(float angle, uint16_t dir, uint16_t pul, bool isAZ = true)
{
  float deltaAngle = angle - (isAZ ? currentPositionAZ() : currentPositionEL());
  
  int steps = (int)((deltaAngle / 360.0) * STEPS_PER_REV); // Calculate steps for the given angle

  if (steps < 0)
  {
    digitalWrite(dir, LOW); // Set direction to counterclockwise
    steps = -steps;         // Convert to positive steps
  }
  else
  {
    digitalWrite(dir, HIGH); // Set direction to clockwise
  }

  float timePerStep = 1.0 / SPEED * 1000; // Time per step in milliseconds
  unsigned long lastStepTime = millis();  // Initialize step timer

  int currentStep = 0;

  while (currentStep < steps)
  {
    unsigned long now = millis();

    if (now - lastStepTime >= timePerStep)
    {
      lastStepTime = now;      // Update last step time
      digitalWrite(pul, HIGH); // Pulse ON
      delayMicroseconds(50);   // Short pulse width
      digitalWrite(pul, LOW);  // Pulse OFF
      currentStep++;           // Increment step count
    }
  }
}

// void rotateToAngleAZ(float angle)
// {
//   float deltaAngle = angle - currentPositionAZ();

//   int steps = (int)((deltaAngle / 360.0) * STEPS_PER_REV); // Calculate steps for the given angle

//   if (steps < 0)
//   {
//     digitalWrite(DIR_AZ, LOW); // Set direction to counterclockwise
//     steps = -steps;            // Convert to positive steps
//   }
//   else
//   {
//     digitalWrite(DIR_EL, HIGH); // Set direction to clockwise
//   }

//   float timePerStep = 1.0 / SPEED * 1000; // Time per step in milliseconds
//   unsigned long lastStepTime = millis();  // Initialize step timer

//   int currentStep = 0;

//   while (currentStep < steps)
//   {
//     unsigned long now = millis();

//     if (now - lastStepTime >= timePerStep)
//     {
//       lastStepTime = now;         // Update last step time
//       digitalWrite(PUL_AZ, HIGH); // Pulse ON
//       delayMicroseconds(50);      // Short pulse width
//       digitalWrite(PUL_AZ, LOW);  // Pulse OFF
//       currentStep++;              // Increment step count
//     }
//   }
// }
