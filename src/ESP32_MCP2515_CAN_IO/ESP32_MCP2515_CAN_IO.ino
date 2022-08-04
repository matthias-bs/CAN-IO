///////////////////////////////////////////////////////////////////////////////////////////////////
// ESP32_MCP2515_CAN_IO.ino
//
// CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module 
// based on ESP32, MCP2515 CAN Controller (+ external CAN Transceiver) and MCP4725 Digital Analog Converter
//
// https://github.com/matthias-bs/CAN-IO
//
// Ressources used:
// - SPI bus (MCP2515 CAN Controller)
// - I2C bus (MCP4725 DAC)
// - 1 GPIO pin (MCP2515 RX Interrupt)
// - 1 GPIO with on-board LED (o.k.: on / error: off) 
// - Analog input pins   (default: 2)
// - Digital input pins  (default: 6)
// - Digital output pins (default: 2)
// - 4ch 3.3V to 5V level shifter for MCP2515 (SCLK, MISO, MOSI, INT)
//   (CS is connected directly - works for me) 
// - 2 MCP4725 DACs 
//
// Notes:
// ------
// - MCP2515 is supplied with 5V (use level shifters for I/O lines!)
// - CAN Transceiver is supplied with 5V
// - MCP4725 is supplied with 5V (pull-ups for SDA and SCL to 3.3V)
// - Routing and decoupling for MCP4725 should be implemented with care
// - Implement voltage dividers and filter capacitors for analog inputs as required
// - Over voltage/over current protection for analog/digital inputs/outputs must be implemented 
//   as required
// - example pin configuration below is for Joy-It SBC-NodeMCU-ESP32
//
// Based on:
// ---------
// MCP_CAN Library for Arduino by Cory Fowler (https://github.com/coryjfowler/MCP_CAN_lib)
// MCP4725 Library by Rob Tillaart (https://github.com/RobTillaart/MCP4725)
// ESP32AnalogRead by Kevin Harrington (https://github.com/madhephaestus/ESP32AnalogRead)
// 
// created: 08/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20220803 Created
//
//
// ToDo: 
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#include <MCP4725.h> // Digital-Analog Converter
#include <Wire.h>    // I2C Bus Interface
#include <mcp_can.h> // CAN Controller
#include <SPI.h>     // SPI Interface

// ESP32 calibrated Analog Input Reading
#include <ESP32AnalogRead.h>

//-----------------------------------------------------------------------------
//
// User Configuration
//

#define _DEBUG_MODE_

// CAN IDs
#define CAN_ID_AOUT0          0x780
#define CAN_ID_AOUT1          0x781
#define CAN_ID_AIN            0x788
#define CAN_ID_DIN            0x78A
#define CAN_ID_DOUT           0x789

// CAN RX Acceptance Filter/Mask
#define CAN_ACC_FILTER        0x780
#define CAN_ACC_MASK          0x7F0

// Timing
#define CAN_BAUDRATE          CAN_500KBPS
#define CAN_CLOCKRATE         MCP_8MHZ
#define I2C_CLOCKRATE         3400000 // 800000 for slower devices
#define T_CYCLE               100     // cycle time in ms
#define T_BUSY                2       // estimated loop processing time 

// Pin configuration
#define PIN_CAN0_INT          27
#define PIN_CAN0_CS           5

// LED is switched on after successful setup;
// then indicates CAN TX status (on: o.k. / off: error)
#define PIN_LED               2

#define PIN_D0_OUT            15
#define PIN_D1_OUT            4

#define PIN_D0_IN             25
#define PIN_D1_IN             26
#define PIN_D2_IN             14
#define PIN_D3_IN             12
#define PIN_D4_IN             34
#define PIN_D5_IN             35

#define PIN_ADC0_IN           32
#define PIN_ADC1_IN           33
//#define PIN_ADC2_IN         34
//#define PIN_ADC3_IN         35

// DAC I2C bus addresses
#define DAC0_ADDR             0x60
#define DAC1_ADDR             0x61

// Analog inputs - voltage dividers and no. of samples per reading
#ifdef PIN_ADC0_IN
    // Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
    const float   ADC0_DIV       = 0.5;       
    const uint8_t ADC0_SAMPLES   = 10;
#endif
#ifdef PIN_ADC1_IN
    // Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
    const float   ADC1_DIV       = 0.5;       
    const uint8_t ADC1_SAMPLES   = 10;
#endif
#ifdef PIN_ADC2_IN
    // Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
    const float   ADC2_DIV       = 0.5;       
    const uint8_t ADC2_SAMPLES   = 10;
#endif
#ifdef PIN_ADC3_IN
    // Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
    const float   ADC3_DIV       = 0.5;       
    const uint8_t ADC3_SAMPLES   = 10;
#endif
//-----------------------------------------------------------------------------

#ifdef PIN_ADC0_IN
    ESP32AnalogRead adc0;
#endif
#ifdef PIN_ADC1_IN
    ESP32AnalogRead adc1;
#endif
#ifdef PIN_ADC2_IN
    ESP32AnalogRead adc2;
#endif
#ifdef PIN_ADC3_IN
    ESP32AnalogRead adc3;
#endif

#define DEBUG_PORT Serial
#if defined(_DEBUG_MODE_)
    #define DEBUG_PRINTF(...) { DEBUG_PORT.printf(__VA_ARGS__); }

#else
    #define DEBUG_PRINTF(...) {}
#endif

//
// Get voltage via ESP32's integrated ADCs
//
// Calculate average over <samples> and apply <divider> to get actual voltage in mV.
//
uint16_t
getVoltage(ESP32AnalogRead &adc, uint8_t samples, float divider)
{
    float voltage_raw = 0;
    for (uint8_t i=0; i < samples; i++) {
        voltage_raw += float(adc.readMiliVolts());
    }
    uint16_t voltage = int(voltage_raw / samples / divider);
     
    //DEBUG_PRINTF("Voltage = %dmV\n", voltage);

    return voltage;
}

#ifdef DAC0_ADDR
MCP4725 MCP0(DAC0_ADDR);
#endif
#ifdef DAC1_ADDR
MCP4725 MCP1(DAC1_ADDR);
#endif

MCP_CAN CAN0(PIN_CAN0_CS);


void setup()
{
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D0_OUT, OUTPUT);
  pinMode(PIN_D1_OUT, OUTPUT);
  pinMode(PIN_CAN0_INT, INPUT);  // Configuring pin for /INT input
  pinMode(PIN_D0_IN, INPUT);
  pinMode(PIN_D1_IN, INPUT);
  pinMode(PIN_D2_IN, INPUT);
  pinMode(PIN_D3_IN, INPUT);
  #ifdef PIN_D4_IN
  pinMode(PIN_D4_IN, INPUT);
  #endif
  #ifdef PIN_D5_IN
  pinMode(PIN_D5_IN, INPUT);
  #endif

  // Switch LED off until setup is completed successfully 
  digitalWrite(PIN_LED, LOW);

  // Initialize MCP2515 running at CAN_CLOCKRATE MHz with a baudrate of CAN_BAUDRATE
  // and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_BAUDRATE, CAN_CLOCKRATE) == CAN_OK) {
    DEBUG_PRINTF("MCP2515 Initialized Successfully!\n");
  }
  else {
    DEBUG_PRINTF("Error Initializing MCP2515...\n");
    while (1);
  }

  // Set CAN RX Filter and Mask
  CAN0.init_Filt(0,0,CAN_ACC_FILTER);
  CAN0.init_Mask(0,0,CAN_ACC_MASK);

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);      

  
  Wire.begin();
  Wire.setClock(3400000);
  
  #ifdef DAC0_ADDR
  MCP0.begin();
  MCP0.setValue(0);
  if (!MCP0.isConnected())
  {
    DEBUG_PRINTF("DAC0 not found!\n");
    while (1);
  }
  #endif
  
  #ifdef DAC1_ADDR
  MCP1.begin();
  MCP1.setValue(0);
  if (!MCP1.isConnected())
  {
    DEBUG_PRINTF("DAC1 not found!\n");
    while (1);
  } 
  #endif
 
  #ifdef PIN_ADC0_IN
      // Use ADC0 with PIN_ADC0_IN
      adc0.attach(PIN_ADC0_IN);
  #endif
  #ifdef PIN_ADC1_IN
      // Use ADC1 with PIN_ADC1_IN
      adc1.attach(PIN_ADC1_IN);
  #endif
  #ifdef PIN_ADC2_IN
      // Use ADC2 with PIN_ADC2_IN
      adc3.attach(PIN_ADC2_IN);
  #endif
  #ifdef PIN_ADC3_IN
      // Use ADC3 with PIN_ADC3_IN
      adc3.attach(PIN_ADC3_IN);
  #endif

  // Setup successful - switch LED on
  digitalWrite(PIN_LED, HIGH);
} // setup()


void loop()
{
  long unsigned int rxId; // CAN RX messafe ID
  unsigned char len = 0;  // CAN RX message length
  unsigned char rxBuf[8]; // CAN RX buffer
  unsigned char txBuf[8]; // CAN TX buffer
  byte          sndStat;  // CAN send status
  uint16_t  u_adc0;       // ADC0 voltage
  uint16_t  u_adc1;       // ADC1 voltage
  uint16_t  u_adc2;       // ADC2 voltage
  uint16_t  u_adc3;       // ADC3 voltage

  // ----------------------
  // Transmit CAN Messages
  // ----------------------

  //
  // Analog Inputs
  //
  #ifdef PIN_ADC0_IN
      u_adc0 = getVoltage(adc0, ADC0_SAMPLES, ADC0_DIV);
  #endif
  #ifdef PIN_ADC1_IN
      u_adc1 = getVoltage(adc1, ADC1_SAMPLES, ADC1_DIV);
  #endif
  #ifdef PIN_ADC2_IN
      u_adc2 = getVoltage(adc2, ADC2_SAMPLES, ADC2_DIV);
  #endif
  #ifdef PIN_ADC3_IN
      u_adc3 = getVoltage(adc3, ADC3_SAMPLES, ADC3_DIV);
  #endif

  // send data:  ID = CAN_ID_AIN, Standard CAN Frame, Data length = 8 bytes
  txBuf[0] =  u_adc0       & 0xFF;
  txBuf[1] = (u_adc0 >> 8) & 0xFF;
  txBuf[2] =  u_adc1       & 0xFF;
  txBuf[3] = (u_adc1 >> 8) & 0xFF;
  txBuf[4] =  u_adc2       & 0xFF;
  txBuf[5] = (u_adc2 >> 8) & 0xFF;
  txBuf[6] =  u_adc3       & 0xFF;
  txBuf[7] = (u_adc3 >> 8) & 0xFF;
  sndStat = CAN0.sendMsgBuf(CAN_ID_AIN, 0, 8, txBuf);
  
  // Show TX status via LED
  if (sndStat == CAN_OK) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }
  
  //
  // Digital Inputs
  //
  txBuf[0] = digitalRead(PIN_D3_IN) << 3 |
             digitalRead(PIN_D2_IN) << 2 |
             digitalRead(PIN_D1_IN) << 1 |
             digitalRead(PIN_D0_IN);
  #ifdef PIN_D4_IN
  txBuf[0] |= digitalRead(PIN_D4_IN) << 4;
  #endif
  #ifdef PIN_D5_IN
  txBuf[0] |= digitalRead(PIN_D5_IN) << 5;
  #endif
  
  // clear remaining TX buffer
  for (int i=1; i<=7; i++) {
    txBuf[i] = 0; 
  }
  sndStat = CAN0.sendMsgBuf(CAN_ID_DIN, 0, 8, txBuf);
  
  // Show TX status via LED
  if (sndStat == CAN_OK) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }


  // ---------------------
  // Receive CAN Messages
  // ---------------------
  if (!digitalRead(PIN_CAN0_INT))             // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    // Generic CAN message decoding
    #if 0
    char msgString[128];    // Array to store serial string
    if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if ((rxId & 0x40000000) == 0x40000000) {    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i<len; i++) {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
    Serial.println();
    #endif

    //
    // Analog Outputs
    //
    #ifdef DAC0_ADDR
    if (rxId  == CAN_ID_AOUT0) {
      uint16_t u_dac0 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut0: 0x%03X\n", u_dac0);
      MCP0.setValue(u_dac0);
    }
    #endif
    
    #ifdef DAC1_ADDR
    if (rxId  == CAN_ID_AOUT1) {
      uint16_t u_dac1 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut1: 0x%03X\n", u_dac1);
      MCP1.setValue(u_dac1);
    }
    #endif

    //
    // Digital Outputs
    //
    if (rxId == CAN_ID_DOUT) {
      DEBUG_PRINTF("DigitalOut: 0x%02X\n", rxBuf[0]);
      digitalWrite(PIN_D0_OUT, rxBuf[0] & 0x01);
      digitalWrite(PIN_D1_OUT, (rxBuf[0] >> 1) & 0x01);
    }
    
  } // if (!digitalRead(PIN_CAN0_INT))
  delay(T_CYCLE-T_BUSY);
}
// -- END OF FILE --
