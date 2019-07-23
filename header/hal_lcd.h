/*
 * hal_spi.h
 *
 *  Created on: Sep 5, 2016
 *      Author: Venugopal Velcheru
 */

#ifndef PUBLIC_HAL_LCD_H_
#define PUBLIC_HAL_LCD_H_

#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 128

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define DELAY 0x80

#define TFT_CS                  (*((volatile uint32_t *)0x40004040))  /* PA4 */
#define TFT_CS_LOW              0x00
#define TFT_CS_HIGH             0x10
#define DC                      (*((volatile uint32_t *)0x40025040))  /* PF4 */
#define DC_COMMAND              0x00
#define DC_DATA                 0x10
#define RESET                   (*((volatile uint32_t *)0x40025004))  /* PF0 */
#define RESET_LOW               0x00
#define RESET_HIGH              0x01

void HAL_LCD_Init(void);
uint16_t HAL_LCD_Color565(uint8_t, uint8_t, uint8_t);

//void hal_LCD_Sendcommnad(uint8_t command);
//void hal_LCD_Senddata(uint8_t data);


#endif /* PUBLIC_HAL_LCD_H_ */
