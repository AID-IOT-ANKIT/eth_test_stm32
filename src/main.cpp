#include "eth_modbus/modbus_ethernet.h"
// Enter an IP address for your controller below.
// The IP address will be dependent on your local network:
IPAddress ip(192, 168, 1, 183);

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // mac

word oldHregVal = 0;

#define HREG 100

#define LED_PIN PB0

ModbusEthernet mb;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  mb.config(mac);
  //   mb.config(mac,ip);

  mb.addHreg(HREG, 127);
  mb.addHreg(HREG + 1, 127);

  //   // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // start the Ethernet connection and the server:
  // Ethernet.begin();
  // server.begin();
}

void loop()
{
  mb.task();

  uint16_t regVal = mb.hreg(HREG);
  uint16_t regVal2 = mb.hreg(HREG + 1);

  if (regVal != oldHregVal)
  {
    oldHregVal = regVal;
    Serial.write("new value:");
    Serial.println(regVal);
    Serial.write("val2:");
    Serial.println(regVal2);

    uint32_t n = (regVal << 16) | regVal2;
    float *f = (float*)&n;
    printf("float %f", *f);
    Serial.println(*f, 2);
    // Serial.println(f);
  }

  delay(30);
}

// #include <Arduino.h>

// // Define LED pin
// #define LED_PIN PB0

// // UART parameters
// #define BAUDRATE 38400
// #define SLAVE_ID 0x01 // Slave ID for this device

// // Buffer to store incoming Modbus frame
// uint8_t modbusBuffer[256];
// uint16_t bufferIndex = 0;

// // Modbus Coil Register
// bool coilState = false;

// HardwareSerial Serial1(PG9, PG14); // D0, D1 of stm32f767zi

// // CRC Calculation Functions
// uint16_t modbusCRC16(const uint8_t *data, uint16_t length)
// {
//   uint16_t crc = 0xFFFF;

//   for (uint16_t i = 0; i < length; i++)
//   {
//     crc ^= data[i];
//     for (uint8_t j = 0; j < 8; j++)
//     {
//       if (crc & 0x0001)
//       {
//         crc = (crc >> 1) ^ 0xA001;
//       }
//       else
//       {
//         crc = crc >> 1;
//       }
//     }
//   }
//   return crc;
// }

// // Function to parse Modbus frame
// void processModbusFrame()
// {
//   // Check if the frame is at least 8 bytes (minimum Modbus RTU frame length)
//   if (bufferIndex < 8)
//   {
//     return;
//   }

//   // Check Slave ID
//   if (modbusBuffer[0] != SLAVE_ID)
//   {
//     return; // Not for this slave
//   }

//   // Validate CRC
//   uint16_t receivedCRC = (modbusBuffer[bufferIndex - 1] << 8) | modbusBuffer[bufferIndex - 2];
//   uint16_t calculatedCRC = modbusCRC16(modbusBuffer, bufferIndex - 2);

//   Serial.print("received crc: ");
//   Serial.println(receivedCRC);

//   Serial.print("calc crc: ");
//   Serial.println(calculatedCRC);

//   if (receivedCRC != calculatedCRC)
//   {
//     return; // CRC mismatch
//   }

//   // Process Modbus Function Code
//   uint8_t functionCode = modbusBuffer[1];
//   if (functionCode == 0x05)
//   { // Write Single Coil
//     uint16_t coilAddress = (modbusBuffer[2] << 8) | modbusBuffer[3];
//     uint16_t coilValue = (modbusBuffer[4] << 8) | modbusBuffer[5];

//     Serial.print("coil addr: ");
//     Serial.println(coilAddress);

//     Serial.print("coil value: ");
//     Serial.println(coilValue);

//     if (coilAddress == 0x0000)
//     {                                    // Coil 0 (LED control)
//       coilState = (coilValue == 0xFF00); // Set coil state
//       digitalWrite(LED_PIN, coilState);
//     }
//   }

//   // Clear buffer after processing
//   bufferIndex = 0;
// }

// void setup()
// {
//   // Initialize UART
//   Serial.begin(115200);
//   Serial1.begin(BAUDRATE, SERIAL_8E1);

//   // Initialize LED pin
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);
// }

// void loop()
// {
//   // Read data from UART
//   while (Serial1.available())
//   {
//     Serial.write(Serial1.read());
//   }

//   while(Serial.available()) {
//     Serial1.write(Serial.read());
//   }

//   // while (Serial1.available())
//   // {
//   //   modbusBuffer[bufferIndex++] = Serial1.read();

//   //   // Process frame if buffer is full or complete frame received
//   //   if (bufferIndex >= sizeof(modbusBuffer) || (bufferIndex > 7 && Serial1.peek() == -1))
//   //   {
//   //     Serial.println("processing buffer");
//   //     processModbusFrame();
//   //   }
//   // }
// }