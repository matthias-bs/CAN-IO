/*
mcp4725 lib 0x01

copyright (c) Davide Gironi, 2018

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include "mcp4725.h"

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>


//path to i2c fleury lib
#include MCP4725_I2CFLEURYPATH


/*
 * initialize
 */
void mcp4725_init(uint8_t address, uint8_t resetic) {
	#if MCP4725_I2CINIT == 1
	//init i2c
	i2c_init();
	_delay_us(10);
	#endif

	//set power down mode
	i2c_start_wait(I2C_WRITE + MCP4725_BASEADDRESS + address);
#if MCP4725_POWERDOWNMODE == MCP4725_POWERDOWN1K
	i2c_write(0b01000010);
#elif MCP4725_POWERDOWNMODE == MCP4725_POWERDOWN100K
	i2c_write(0b01000100);
#elif MCP4725_POWERDOWNMODE == MCP4725_POWERDOWN500K
	i2c_write(0b01000110);
#endif
	if(resetic) {
		i2c_write(0b00000000);
		i2c_write(0b00000000);
	}
	i2c_stop();
}

/*
 * set the raw value to output fast mode
 */
void mcp4725_setrawoutputfast(uint8_t address, uint16_t rawoutput) {
	//write raw output value to register
	i2c_start_wait(I2C_WRITE + MCP4725_BASEADDRESS + address);
	i2c_write((uint8_t)(rawoutput>>8));
	i2c_write((uint8_t)(rawoutput));
	i2c_stop();
}

/*
 * set a voltage value to output fast mode
 */
void mcp4725_setvoltagefast(uint8_t address, double voltage, double dacref) {
	uint16_t rawoutput = 0;

	//get the raw output
	rawoutput = voltage*4096/dacref;
	if (rawoutput > 4095)
		rawoutput = 4095;

	//write to chip
	mcp4725_setrawoutputfast(address, rawoutput);
}

/*
 * set the raw value to output
 */
void mcp4725_setrawoutput(uint8_t address, uint16_t rawoutput, uint8_t savetoeeprom) {
	//write raw output value to register
	i2c_start_wait(I2C_WRITE + MCP4725_BASEADDRESS + address);
	if(savetoeeprom) {
		i2c_write(MCP4725_WRITEDACEEPROM);
	} else {
		i2c_write(MCP4725_WRITEDAC);
	}
	i2c_write((uint8_t)(rawoutput>>4));
	rawoutput = (rawoutput<<12);
	i2c_write((uint8_t)(rawoutput>>8));
	i2c_stop();
}

/*
 * set a voltage value to output
 */
void mcp4725_setvoltage(uint8_t address, double voltage, double dacref, uint8_t savetoeeprom) {
	uint16_t rawoutput = 0;

	//get the raw output
	rawoutput = voltage*4096/dacref;
	if (rawoutput > 4095)
		rawoutput = 4095;

	//write to chip
	mcp4725_setrawoutput(address, rawoutput, savetoeeprom);
}
