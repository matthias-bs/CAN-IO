/*
mcp4725 lib 0x01

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include <avr/io.h>


#ifndef MCP4725_H_
#define MCP4725_H_


#define MCP4725_BASEADDRESS (0xC0) //device base address

//i2c settings
#define MCP4725_I2CFLEURYPATH "../i2chw/i2cmaster.h" //define the path to i2c fleury lib
#define MCP4725_I2CINIT 1 //init i2c

//registers
#define MCP4725_WRITEDAC 0x40
#define MCP4725_WRITEDACEEPROM 0x60
#define MCP4725_READDAC 0x03

//set the power down mode
#define MCP4725_POWERDOWN1K 1
#define MCP4725_POWERDOWN100K 100
#define MCP4725_POWERDOWN500K 500
#define MCP4725_POWERDOWNMODE MCP4725_POWERDOWN1K

//functions
extern void mcp4725_init(uint8_t address, uint8_t resetic);
extern void mcp4725_setrawoutputfast(uint8_t address, uint16_t rawoutput);
extern void mcp4725_setvoltagefast(uint8_t address, double voltage, double dacref);
extern void mcp4725_setrawoutput(uint8_t address, uint16_t rawoutput, uint8_t savetoeeprom);
extern void mcp4725_setvoltage(uint8_t address, double voltage, double dacref, uint8_t savetoeeprom);

#endif
