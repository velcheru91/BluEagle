/*
 * hal_spi.c
 *
 *  Created on: Sep 5, 2016
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
//#include "sys_time.h"
#include "hal.h"
//#include "sys_interrupts.h"
#include "hal_lcd.h"
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void hal_spiWrite(uint8_t data)
{
	SSI2_DR_R = data;			// write command
	while (SSI2_SR_R & SSI_SR_BSY);
}

void hal_LCD_csOn()
{
	HAL_LCD_CS = 0; 			// assert chip select
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	__asm (" NOP");
}

void hal_LCD_csOff()
{
	HAL_LCD_CS = 1;				// de-assert chip select
}
// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void hal_LCD_Sendcommnad(uint8_t command)
{
	hal_LCD_csOn();
	hal_spiWrite((uint16_t) command);
	hal_LCD_csOff();

}

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void hal_LCD_Senddata(uint8_t data)
{
	uint16_t RAM_data = (0x0100 | (uint16_t) data);
	hal_LCD_csOn();
	hal_spiWrite(RAM_data);
	hal_LCD_csOff();

}
