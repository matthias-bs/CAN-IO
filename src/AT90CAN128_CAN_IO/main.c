///////////////////////////////////////////////////////////////////////////////////////////////////
// AT90CAN128_CAN_IO
//
// CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module 
// based on AT90CAN128 (+ external CAN Transceiver) and MCP4725 Digital Analog Converter
//
// https://github.com/matthias-bs/CAN-IO
//
// Ressources used:
// - 2 GPIO pins (CAN TX & RX)
// - I2C bus (MCP4725 DAC)
// - 1 GPIO with on-board LED (o.k.: on / error: off) 
// - Analog input pins   (default: 4)
// - Digital input pins  (default: 8)
// - Digital output pins (default: 8)
// - 2 MCP4725 DACs 
//
// Notes:
// ------
// - Supply voltage is 5V for MCU, CAN transceiver and DACs
// - I2C bus has external pull-ups to 5V
// - Internal pull-ups for digital input ports are enabled
// - Routing and decoupling for MCP4725 should be implemented with care
// - Wiring and decoupling of MCU's analog pins should be implemented according to manufacturer's
//   recommendations
// - Implement voltage dividers and filter capacitors for analog inputs as required
// - Over voltage/over current protection for analog/digital inputs/outputs must be implemented 
//   as required
// - example is based on Crumb128-CAN (http://www.chip45.com)
//
// Based on:
// ---------
// AT90CANLib by Chris Blust (https://github.com/chblust/AT90CANLib)
// TWI master library by Peter Fleury
// mcp4725 AVR library by Davide Gironi (https://sourceforge.net/projects/davidegironi/files/avr-lib/)
// adc driver by Davide Gironi (https://sourceforge.net/projects/davidegironi/files/avr-lib/) (mcp4725 archive)
// AVR-UART-lib by jnk0le (https://github.com/jnk0le)
//
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
// 20220808 Created
//
//
// ToDo: 
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart/usart.h"
#include "adc/adc.h"
#include "mcp4725/mcp4725.h"
#include "can/can.h"


#define _DEBUG_MODE_
//#define DEBUG_ADC


// CAN IDs
// Note: AT90CANLib does support provide Extended IDs (29 bits) yet!
#define CAN_ID_AOUT0    0x780
#define CAN_ID_AOUT1    0x781
#define CAN_ID_AIN      0x788
#define CAN_ID_DIN      0x78A
#define CAN_ID_DOUT     0x789

// DAC and ADC configuration
#define DAC0_ADDR       0
#define DAC1_ADDR       1
#define ADC0_CH         0
#define ADC1_CH         1
#define ADC2_CH         2
#define ADC3_CH         3

// Port configuration
#define DIN_PORT        PORTA   // digital inputs - port/pull-up register
#define DIN_PIN         PINA    // digital inputs - input register
#define DIN_DDR         DDRA    // digital inputs - data direction register
#define DOUT_PORT       PORTC   // output register
#define DOUT_DDR        DDRC    // data direction register

// Timing
#define T_CYCLE         100     // cycle time in ms
#define T_BUSY          2       // estimated loop processing time


// Analog inputs - reference voltage, voltage dividers and no. of samples per reading

// Reference voltage - 2.56V internal bandgap reference; can be measured at AREF pin
const float   ADC_VREF       = 2560.0;

// Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
const float   ADC0_DIV       = 0.5;       
const uint8_t ADC0_SAMPLES   = 10;

// Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
const float   ADC1_DIV       = 0.5;       
const uint8_t ADC1_SAMPLES   = 10;

// Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
const float   ADC2_DIV       = 0.5;       
const uint8_t ADC2_SAMPLES   = 10;

// Voltage divider R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
const float   ADC3_DIV       = 0.5;       
const uint8_t ADC3_SAMPLES   = 10;


#if defined(_DEBUG_MODE_)
    #define DEBUG_PUTS(str) { uart_puts((str)); }

#else
    #define DEBUG_PUTS(str) {}
#endif


static inline void led_init(void)
{
    // Enable LED (connected to VCC) output on Port B, bit 7
    DDRB |= _BV(PB7);
}

static inline void led_on(void)
{
    PORTB &= 0x7F;
}

static inline void led_off(void)
{
    PORTB |= 0x80;
}


//
// Get voltage via MCU's integrated ADCs
//
// Calculate average over <samples> and apply <divider> to get actual voltage in mV.
//
// channel - ADC channel no.
// samples - no. of samples for averaging
// divider - voltage divider: R1 / (R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)
// vref    - reference voltage in mV
//
uint16_t
getVoltage(uint8_t channel, uint8_t samples, float divider, float vref)
{
    float    voltage_raw = 0;
    
    for (uint8_t i=0; i < samples; i++) {
        voltage_raw += (float)(adc_read(channel) * vref/1024);
    }
    uint16_t voltage = (int)(voltage_raw / samples / divider);
    
#ifdef DEBUG_ADC
    //DEBUG_PRINTF("Voltage = %dmV\n", voltage);
    char printbuff[16];
    itoa(channel, printbuff, 10);
    uart_puts("DAC"); 
    uart_puts(printbuff); 
    uart_puts(": "); 
    itoa(voltage, printbuff, 10);
    uart_puts(printbuff); uart_puts("\r\n");
#endif
    
    return voltage;
}

int main(void) {
    
    led_init();
    led_off();
    
    // init inputs (with pull-ups)
    DIN_DDR  = 0x00;
    DIN_PORT = 0xFF;
    
    // init outputs
    DOUT_PORT = 0x00;
    DOUT_DDR  = 0xFF;
    
    // init UART
    uart_init(BAUD_CALC(UART_BAUDRATE)); // 8n1 transmission is set as default

    // init ADC
    // Notes: 
    // ADC clock must be in the range 50..200 kHz; 16MHz / 256 = 125 kHz
    // see configuration in adc.h
    // Please check AT90CAN User Manual for wiring of analog pins!
    adc_init();  
    
    DEBUG_PUTS("Initializing DAC0\n");
    mcp4725_init(DAC0_ADDR, 0);
    mcp4725_setrawoutputfast(DAC0_ADDR, 0);
    
    DEBUG_PUTS("Initializing DAC1\n");
    mcp4725_init(DAC1_ADDR, 0);
    mcp4725_setrawoutputfast(DAC1_ADDR, 0);

    
    CANMessage rx_msg;
    CANMessage tx_msg;
    uint8_t    res;
    
    initCAN();
    
    // Init for CAN RX Message CAN_ID_AOUT0, DLC=8
    res = listenForMessage(CAN_ID_AOUT0, 8);
    if (res == 0) {
        DEBUG_PUTS("CAN_ID_AOUT0 CAN message initialization failed!\n");
        while (1);
    }
    
    // Init for CAN RX Message CAN_ID_AOUT1, DLC=8
    res = listenForMessage(CAN_ID_AOUT1, 8);
    if (res == 0) {
        DEBUG_PUTS("CAN_ID_AOUT1 CAN message initialization failed!\n");
        while (1);
    }
    
    // Init for CAN RX Message CAN_ID_DOUT, DLC=8
    res = listenForMessage(CAN_ID_DOUT, 8);
    if (res == 0) {
        DEBUG_PUTS("CAN_ID_DOUT CAN message initialization failed!\n");
        while (1);
    }
   
    
    led_on();
    
    for(;;) {
        // ----------------------
        // Transmit CAN Messages
        // ----------------------

        
        //
        // Analog Inputs
        //
        uint16_t u_adc0 = getVoltage(ADC0_CH, ADC0_SAMPLES, ADC0_DIV, ADC_VREF);
        uint16_t u_adc1 = getVoltage(ADC1_CH, ADC1_SAMPLES, ADC1_DIV, ADC_VREF);
        uint16_t u_adc2 = getVoltage(ADC2_CH, ADC2_SAMPLES, ADC2_DIV, ADC_VREF);
        uint16_t u_adc3 = getVoltage(ADC3_CH, ADC3_SAMPLES, ADC3_DIV, ADC_VREF);
        
        tx_msg.id     = CAN_ID_AIN;
        tx_msg.length = 8;
        for (uint8_t i=0; i < 8; i++) {
            tx_msg.data[i] = 0;
        }
        tx_msg.data[0] =  u_adc0       & 0xFF;
        tx_msg.data[1] = (u_adc0 >> 8) & 0xFF;
        tx_msg.data[2] =  u_adc1       & 0xFF;
        tx_msg.data[3] = (u_adc1 >> 8) & 0xFF;
        tx_msg.data[4] =  u_adc2       & 0xFF;
        tx_msg.data[5] = (u_adc2 >> 8) & 0xFF;
        tx_msg.data[6] =  u_adc3       & 0xFF;
        tx_msg.data[7] = (u_adc3 >> 8) & 0xFF;
        res = sendCAN(&tx_msg);
        if (res == 0) {
            DEBUG_PUTS("AnalogIn CAN message TX failed!\n");
            led_off();      
        } else {
            led_on();
        }

        
        //
        // Digital Inputs
        //
        tx_msg.id     = CAN_ID_DIN;
        tx_msg.length = 8;
        for (uint8_t i=0; i < 8; i++) {
            tx_msg.data[i] = 0;
        }
        tx_msg.data[0] = DIN_PIN;
        res = sendCAN(&tx_msg);
        if (res == 0) {
            DEBUG_PUTS("DigitalIn CAN message TX failed!\n");
            led_off();      
        } else {
            led_on();
        }
        

        // ---------------------
        // Receive CAN Messages
        // ---------------------
        if (getMessage(&rx_msg)) {
            
            //
            // Analog Outputs
            //
            if (rx_msg.id == CAN_ID_AOUT0) {
                uint16_t data = rx_msg.data[1] << 8 | rx_msg.data[0];
                mcp4725_setrawoutputfast(DAC0_ADDR, data);
                #ifdef _DEBUG_MODE_
                    char buf[17];
                    itoa(data, buf, 16);
                #endif

                DEBUG_PUTS("AnalogOut0: 0x");
                DEBUG_PUTS(buf);
                DEBUG_PUTS("\n");
            }
            if (rx_msg.id == CAN_ID_AOUT1) {
                uint16_t data = rx_msg.data[1] << 8 | rx_msg.data[0];
                mcp4725_setrawoutputfast(DAC1_ADDR, data);
                #ifdef _DEBUG_MODE_
                    char buf[17];
                    itoa(data, buf, 16);
                #endif
                DEBUG_PUTS("AnalogOut1: 0x");
                DEBUG_PUTS(buf);
                DEBUG_PUTS("\n");
            }
            
            //
            // Digital Outputs
            //            
            if (rx_msg.id == CAN_ID_DOUT) {
                uint16_t data = rx_msg.data[0];
                DOUT_PORT = data;
                #ifdef _DEBUG_MODE_
                    char buf[17];
                    itoa(data, buf, 16);
                #endif
                DEBUG_PUTS("DigitalOut: 0x");
                DEBUG_PUTS(buf);
                DEBUG_PUTS("\n");
            }
            
        }
       _delay_ms(T_CYCLE-T_BUSY);
    } // for(;;)
    
}
