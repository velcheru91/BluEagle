/*
 * hal_spi.c
 *
 *  Created on: Sep 5, 2016
 *      Author: Venugopal Velcheru
 */

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#ifndef _STDINT_H_
#include <stdint.h>
#endif
#ifndef __TM4C123GH6PM_H__
#include <tm4c123gh6pm.h>
#endif
#ifndef _STDBOOL
#include <stdbool.h>
#endif
#ifndef PUBLIC_HAL_H_
#include <hal.h>
#endif
#ifndef PUBLIC_HAL_LCD_H_
#include <hal_lcd.h>
#endif
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
/*----------------------------------------------------------*/
enum initRFlags{
  none,
  INITR_GREENTAB,
  INITR_REDTAB,
  INITR_BLACKTAB
};
static const uint8_t
  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 };                 //     16-bit color
static const uint8_t
  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x7F+0x01 };      //     XEND = 127
//static const uint8_t
//  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
//    2,                        //  2 commands in list:
//    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
//      0x00, 0x00,             //     XSTART = 0
//      0x00, 0x7F,             //     XEND = 127
//    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
//      0x00, 0x00,             //     XSTART = 0
//      0x00, 0x7F };           //     XEND = 127
static const uint8_t
  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay

uint8_t static spiwritecommand(uint8_t c) {
                                        // wait until SSI2 not busy/transmit FIFO empty
	while((SSI2_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
	TFT_CS = TFT_CS_LOW;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	DC = DC_COMMAND;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	SSI2_DR_R = c;                        // data out
	while((SSI2_SR_R&SSI_SR_RNE)==0){};   // wait until response
	TFT_CS = TFT_CS_HIGH;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	return (uint8_t)SSI2_DR_R;            // return the response
}

uint8_t static spiwritedata(uint8_t d) {
                                        // wait until SSI2 not busy/transmit FIFO empty
	while((SSI2_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
	TFT_CS = TFT_CS_LOW;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	DC = DC_DATA;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	SSI2_DR_R = d;                        // data out
	while((SSI2_SR_R&SSI_SR_RNE)==0){};   // wait until response
	TFT_CS = TFT_CS_HIGH;
	__asm (" NOP");             // allow line to settle
	__asm (" NOP");
	return (uint8_t)SSI2_DR_R;            // return the response
}
// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in ROM byte array.
void static commandList(const uint8_t *addr) {

	uint8_t numCommands, numArgs;
	uint16_t ms;

	numCommands = *(addr++);               // Number of commands to follow
	while(numCommands--)
	{                 // For each command...
		spiwritecommand(*(addr++));             //   Read, issue command
		numArgs  = *(addr++);                //   Number of args to follow
		ms       = numArgs & DELAY;          //   If hibit set, delay follows args
		numArgs &= ~DELAY;                   //   Mask out delay bit
		while(numArgs--)
		{                   //   For each argument...
			spiwritedata(*(addr++));              //     Read, issue argument
		}

		if(ms)
		{
			ms = *(addr++);             // Read post-command delay time (ms)
			if(ms == 255) ms = 500;     // If 255, delay for 500 ms
			HAL_Sys_Delay(DELAY_1mSEC*ms);
		}
	}
}

// Initialization code common to both 'B' and 'R' type displays
void static commonInit() {
//	ColStart  = RowStart = 0; // May be overridden in init func

	// toggle RST low to reset; CS low so it'll listen to us
	// SSI2Fss is not available, so use GPIO on PA4

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;  // 2a) unlock GPIO Port F
	GPIO_PORTF_CR_R = HAL_PORT_ENABLE;          // allow changes to PF4-0
                                   // 2b) no need to unlock PF4, PB7, PB4, or PA4
	GPIO_PORTF_AMSEL_R &= ~(HAL_GPIO_BIT4 | HAL_GPIO_BIT0);     // 3a) disable analog on PF4,0
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0FFF0)+0x00000000;
                                   // 4b) configure PB7,4 as SSI
	GPIO_PORTF_DIR_R |= (HAL_GPIO_BIT4 | HAL_GPIO_BIT0);        // 5a) make PF4,0 output
	GPIO_PORTF_AFSEL_R &= ~(HAL_GPIO_BIT4 | HAL_GPIO_BIT0);     // 6a) disable alt funct on PF4,0
	GPIO_PORTF_DEN_R |= (HAL_GPIO_BIT4 | HAL_GPIO_BIT0);        // 7a) enable digital I/O on PF4,0



	GPIO_PORTB_AMSEL_R &= ~0x90;     // 3b) disable analog on PB7,4
	GPIO_PORTB_AFSEL_R |= 0x90;      // 6b) enable alt funct on PB7,4
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x0FF0FFFF)+0x20020000;
	GPIO_PORTB_DEN_R |= 0x90;        // 7b) enable digital I/O on PB7,4
	                                   // 4c) configure PA4 as GPIO
	GPIO_PORTA_AMSEL_R &= ~0x10;     // 3c) disable analog on PA4
                                   // 4a) configure PF4,0 as GPIO
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFF0FFFF)+0x00000000;
	GPIO_PORTA_DIR_R |= 0x10;        // 5b) make PA4 output
	GPIO_PORTA_AFSEL_R &= ~0x10;     // 6c) disable alt funct on PA4
	GPIO_PORTA_DEN_R |= 0x10;        // 7c) enable digital I/O on PA4

	TFT_CS = TFT_CS_LOW;
	RESET = RESET_HIGH;
	HAL_Sys_Delay(DELAY_500mSEC);
	RESET = RESET_LOW;
	HAL_Sys_Delay(DELAY_500mSEC);
	RESET = RESET_HIGH;
	HAL_Sys_Delay(DELAY_500mSEC);
	TFT_CS = TFT_CS_HIGH;

	// initialize SSI2
                                        	// activate clock for SSI2
	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
                                        	// allow time for clock to stabilize
	while((SYSCTL_PRSSI_R&SYSCTL_PRSSI_R2) == 0){};
	SSI2_CR1_R &= ~SSI_CR1_SSE;           // disable SSI
	SSI2_CR1_R &= ~SSI_CR1_MS;            // master mode
                                        	// configure for clock from source PIOSC for baud clock source
	SSI2_CC_R = (SSI2_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_PIOSC;
                                        // clock divider for 4 MHz SSIClk (16 MHz PIOSC/4)
                                        // PIOSC/(CPSDVSR*(1+SCR))
                                        // 16/(4*(1+0)) = 4 MHz
	SSI2_CPSR_R = (SSI2_CPSR_R&~SSI_CPSR_CPSDVSR_M)+4; // must be even number
	SSI2_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (4 Mbps data rate)
                  SSI_CR0_SPH |         // SPH = 0
                  SSI_CR0_SPO);         // SPO = 0
                                        // FRF = Freescale format
	SSI2_CR0_R = (SSI2_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
                                        // DSS = 8-bit data
	SSI2_CR0_R = (SSI2_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;
	SSI2_CR1_R |= SSI_CR1_SSE;            // enable SSI

}


//------------ST7735_InitR------------
// Initialization for ST7735R screens (green or red tabs).
// Input: option one of the enumerated options depending on tabs
// Output: none
void static ST7735_InitR() {
	commonInit();
	commandList(Rcmd1);
//	if(option == INITR_GREENTAB)
//	{
		commandList(Rcmd2green);
//		ColStart = 2;
//		RowStart = 3;
//	}
//	else
//	{
//		// colstart, rowstart left at default '0' values
//		commandList(Rcmd2red);
//	}
	commandList(Rcmd3);

	// if black, change MADCTL color filter
//	if (option == INITR_BLACKTAB)
//	{
//    writecommand(ST7735_MADCTL);
//    writedata(0xC0);
//	}
//  TabColor = option;
//	BSP_LCD_SetCursor(0,0);
//	StTextColor = ST7735_YELLOW;
//	BSP_LCD_FillScreen(0);                // set screen to black
}

//------------BSP_LCD_Color565------------
// Pass 8-bit (each) R,G,B and get back 16-bit packed color.
// Input: r red value
//        g green value
//        b blue value
// Output: 16-bit color
uint16_t HAL_LCD_Color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
// Set the region of the screen RAM to be modified
// Pixel colors are sent left to right, top to bottom
// (same as Font table is encoded; different from regular bitmap)
// Requires 11 bytes of transmission
void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

  spiwritecommand(ST7735_CASET); // Column addr set
  spiwritedata(0x00);
  spiwritedata(x0+2);     // XSTART ColStart = 2
  spiwritedata(0x00);
  spiwritedata(x1+2);     // XEND

  spiwritecommand(ST7735_RASET); // Row addr set
  spiwritedata(0x00);
  spiwritedata(y0+3);     // YSTART RowStart = 3
  spiwritedata(0x00);
  spiwritedata(y1+3);     // YEND

  spiwritecommand(ST7735_RAMWR); // write to RAM
}
void HAL_LCD_Init(void)
{
	uint8_t x,y;
	ST7735_InitR();
	setAddrWindow(0,0,127,127);
	  for(y=128; y>0; y--) {
	    for(x=128; x>0; x--) {
	      spiwritedata(0x00);
	      spiwritedata(0x1F);
	    }
	  }
}
