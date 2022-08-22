///////////////////////////////////////////////////////////////////////////////////////////////////
// ATSAME51_CAN_IO.ino
//
// CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module 
// based on AT SAME51 (and optional MCP4725 Digital Analog Converters)
//
// https://github.com/matthias-bs/CAN-IO
//
// Ressources used:
// - 4 GPIO pins (CAN TX & RX, CAN_STANDBY, CAN_BOOSTEN)
// - I2C bus (MCP4725 DAC)
// - 1 GPIO with on-board LED (o.k.: on / error: off) 
// - Analog input pins   (default: 4)
// - Analog output pins: 2 (or 2 MCP4725 DACs)
// - Digital input pins  (default: 6)
// - Digital output pins (default: 2) 
//
// Notes:
// ------
// - Implemented for Adafruit Feather M4 Express CAN
// - On board CAN Transceiver with power-supply
// - MCP4725 is supplied with 5V (pull-ups for SDA and SCL to 3.3V)
// - Routing and decoupling for MCP4725 should be implemented with care
// - If internal DACs are used and 0..5 V outputs are required, external OpAmps are required  
// - Implement voltage dividers and filter capacitors for analog inputs as required
// - Over voltage/over current protection for analog/digital inputs/outputs must be implemented 
//   as required
//
// Based on:
// ---------
// Arduino CAN Library by Adafruit (https://github.com/adafruit/arduino-CAN)
// MCP4725 Library by Rob Tillaart (https://github.com/RobTillaart/MCP4725)
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
// 20220822 Created
//
//
// ToDo: 
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#include <MCP4725.h> // Digital-Analog Converter
#include <Wire.h>    // I2C Bus Interface
#include <CAN.h>     // ESP32 integrated CAN Controller


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
#define CAN_BAUDRATE          500E3
#define I2C_CLOCKRATE         3400000 // 800000 for slower devices
#define T_CYCLE               100     // cycle time in ms
#define T_BUSY                2       // estimated loop processing time 

// Pin configuration

// Note: The CAN transceiver pins are already defined in Adafruit's feather_m4_can header file
//#define PIN_CAN_STANDBY       40
//#define PIN_CAN_BOOSTEN       41
//#define PIN_CAN_TX            42
//#define PIN_CAN_RX            43

// LED is switched on after successful setup;
// then indicates CAN TX status (on: o.k. / off: error)
// Note: PIN_LED is defined in Adafruit's feather_m4_can header file
//#define PIN_LED               13

#define PIN_D0_OUT            0
#define PIN_D1_OUT            1

#define PIN_D0_IN             5
#define PIN_D1_IN             6
#define PIN_D2_IN             9
#define PIN_D3_IN             10
#define PIN_D4_IN             11
#define PIN_D5_IN             12

// Uncomment for using internal DACs
#define PIN_DAC0_OUT        PIN_DAC0
#define PIN_DAC1_OUT        PIN_DAC1

#define PIN_ADC0_IN         A2
#define PIN_ADC1_IN         A3
#define PIN_ADC2_IN         A4
#define PIN_ADC3_IN         A5

// DAC I2C bus addresses
// Uncomment for using external DACs (MCP4725)
//#define DAC0_ADDR             0x60
//#define DAC1_ADDR             0x61

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

//#define DEBUG_PORT  SERIAL_PORT_USBVIRTUAL
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
getVoltage(int pin, uint8_t samples, float divider)
{
    float voltage_raw = 0;
    for (uint8_t i=0; i < samples; i++) {
        voltage_raw += float(analogRead(pin));
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


// CAN receive callback
void onReceive(int packetSize) {
  long unsigned int rxId; // CAN RX messafe ID
  int len = 0;            // CAN RX message length
  unsigned char rxBuf[8]; // CAN RX buffer

  if (!CAN.packetRtr())
  {
    len  = packetSize;
    rxId = CAN.packetId();
    for (int i=0; i<packetSize;i++) {
      rxBuf[i] = CAN.read();
    }

    //
    // Analog Outputs
    //
    #ifdef DAC0_ADDR
    if (rxId == CAN_ID_AOUT0) {
      uint16_t u_dac0 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut0: 0x%03X\n", u_dac0);
      MCP0.setValue(u_dac0);
    }
    #endif

    #ifdef PIN_DAC0_OUT
    if (rxId == CAN_ID_AOUT0) {
      uint16_t u_dac0 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut0: 0x%03X\n", u_dac0);
      analogWrite(PIN_DAC0_OUT, u_dac0);
    }      
    #endif
    
    #ifdef DAC1_ADDR
    if (rxId == CAN_ID_AOUT1) {
      uint16_t u_dac1 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut1: 0x%03X\n", u_dac1);
      MCP1.setValue(u_dac1);
    }
    #endif
    
    #ifdef PIN_DAC1_OUT
    if (rxId == CAN_ID_AOUT1) {
      uint16_t u_dac1 = (rxBuf[1] & 0xF) << 8 | rxBuf[0];
      DEBUG_PRINTF("AnalogOut1: 0x%03X\n", u_dac1);
      analogWrite(PIN_DAC1_OUT, u_dac1);
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
    
  } // if (!CAN.packetRtr())
}

void setup()
{
  #if defined(_DEBUG_MODE_)
    DEBUG_PORT.begin(115200);
    while (!DEBUG_PORT);
  #endif
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_D0_OUT, OUTPUT);
  pinMode(PIN_D1_OUT, OUTPUT);
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

  // Enable CAN transceiver
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster


  // start the CAN bus at 500 kbps
  if (!CAN.begin(CAN_BAUDRATE)) {
    DEBUG_PRINTF("Starting CAN failed!\n");
    while (1);
  }
  
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
 
  // Set CAN RX Acceptance Filter & Mask
  CAN.filter(CAN_ACC_FILTER, CAN_ACC_MASK);
  
  // register the CAN receive callback
  CAN.onReceive(onReceive);

  // Configure Analog I/O
  analogReference(AR_DEFAULT);
  analogReadResolution(12);
  
  // Setup successful - switch LED on
  digitalWrite(PIN_LED, HIGH);
} // setup()


void loop()
{
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
      u_adc0 = getVoltage(PIN_ADC0_IN, ADC0_SAMPLES, ADC0_DIV);
  #endif
  #ifdef PIN_ADC1_IN
      u_adc1 = getVoltage(PIN_ADC1_IN, ADC1_SAMPLES, ADC1_DIV);
  #endif
  #ifdef PIN_ADC2_IN
      u_adc2 = getVoltage(PIN_ADC2_IN, ADC2_SAMPLES, ADC2_DIV);
  #endif
  #ifdef PIN_ADC3_IN
      u_adc3 = getVoltage(PIN_ADC3_IN, ADC3_SAMPLES, ADC3_DIV);
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
  CAN.beginPacket(CAN_ID_AIN, 8);
  CAN.write(txBuf, 8);
  sndStat = CAN.endPacket();
  
  // Show TX status via LED
  if (sndStat) {
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
  CAN.beginPacket(CAN_ID_DIN, 8);
  CAN.write(txBuf, 8);
  sndStat = CAN.endPacket();
  
  // Show TX status via LED
  if (sndStat) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  delay(T_CYCLE-T_BUSY);
}
// -- END OF FILE --
