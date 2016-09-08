/*
 * temp.c
 *
 *  Created on: Aug 26, 2016
 *      Author: code_it
 */



//void initHw()
//{
//	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
//    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
//    SYSCTL_GPIOHBCTL_R = 0;
//    // Enable GPIO port A and F peripherals
//    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD;
//    // Configure LED and pushbutton pins
//    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs
//    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
//    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
//    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button
//    // Configure UART0 pins
//	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
//    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
//	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
//    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
//   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
//    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
//	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
//    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
//    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
//    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
//    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
//    // Configure UART1 pins
//    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
//    GPIO_PORTC_DIR_R |= 0x40;
//    GPIO_PORTC_DEN_R |= 0x70;
//    GPIO_PORTC_AFSEL_R = 0x30;
//    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
//    // Configure UART1 to 38400 baud, 811 format (must be 3 clocks from clock enable and config writes)
//    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
//    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
//    UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx38400), set floor(r)=65, where N=16
//    UART1_FBRD_R = 7;                               // round(fract(r)*64)=7
//    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN; // configure for 811 w/ FIFO disable
//    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
//    UART1_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
//    NVIC_EN0_R = 1<<6;
//
//    // Configure FREQ_IN for frequency counter
//    GPIO_PORTD_DEN_R |= 0x01;                        // enable bit 0 for digital input
//    GPIO_PORTD_AFSEL_R |= 0x01;                      // select alternative functions for FREQ_IN pin
//    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;           // map alt fns to FREQ_IN clear
//    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;
//    // Configure WideTimer 2 as the time base
//    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;       // turn-on timer
//    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
//    WTIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
//    WTIMER2_TAMR_R =TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
//    WTIMER2_TAILR_R = 0x61A80;                     // set load value to (40e6/100) for 100 Hz interrupt rate
//    WTIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
//    NVIC_EN3_R |= 1 << (INT_WTIMER2A-96-16);             // turn-on interrupt 114 (WTIMER1A)
//    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
//
//    //SYSCTL_RCGCEEPROM_R = 0x01;
//    //__asm(" NOP");
//    //__asm(" NOP");
//    //__asm(" NOP");
//    //__asm(" NOP");
//    //while(EEPROM_EEDONE_R & 0x01);
//    //EEPROM_EEBLOCK_R = 0x1;
//    //EEPROM_EEOFFSET_R = 0x9;
//    ConfgRom();
//    Send_SRC_ADD = EEPROM_EERDWR_R ;
//void putcUart0(char c)
//{
//	while (UART0_FR_R & UART_FR_TXFF);
//	UART0_DR_R = c;
//	while (UART0_FR_R & UART_FR_BUSY);
//}
//void putsUart0(char* str)
//{
//	int i;
//    for (i = 0; i < strlen(str); i++)
//	  putcUart0(str[i]);
//}
//char getcUart0()
//{
//	while (UART0_FR_R & UART_FR_RXFE);
//	return UART0_DR_R;
//}
//void putaddr_Uart1(char d)
//{
//	EPS = 0;
//	while (UART1_FR_R & UART_FR_TXFF);
//		UART1_DR_R = d;
//		while(UART1_FR_R & UART_FR_BUSY);	    //Checking for UART busy
//}
//void putdata_Uart1(char d)
//{
//	EPS = 1;
//	while (UART1_FR_R & UART_FR_TXFF);
//	UART1_DR_R = d;
//	while(UART1_FR_R & UART_FR_BUSY);	    //Checking for UART busy
//}//
//}

    // Configure SSI2 pins for SPI configuration
//    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
//    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
//    GPIO_PORTB_DR2R_R |= 0x90;                 // set drive strength to 2mA
//	GPIO_PORTB_AFSEL_R |= 0x90;                      // select alternative functions for MOSI, SCLK pins
//    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
//    GPIO_PORTB_DEN_R |= 0x90;                        // enable digital operation on TX, CLK pins

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
// turn off SSI2 to allow re-configuration
//    SSI2_CR1_R &= ~SSI_CR1_SSE;
//    // select master mode
//    SSI2_CR1_R = 0;
//    // select system clock as the clock source
//    SSI2_CC_R = 0;
//    // set bit rate to 1 MHz (if SR=0 in CR0)
//    SSI2_CPSR_R = 40;
//    // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
//    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;
//    // turn on SSI2
//    SSI2_CR1_R |= SSI_CR1_SSE;



// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
//void sendGraphicsLcdCommand(uint8_t command)
//{
//	CS_NOT = 0;                        // assert chip select
//	__asm (" NOP");                    // allow line to settle
//	__asm (" NOP");
//	__asm (" NOP");
//	__asm (" NOP");
//	A0 = 0;                            // clear A0 for commands
//	SSI2_DR_R = command;               // write command
//	while (SSI2_SR_R & SSI_SR_BSY);
//	CS_NOT = 1;                        // de-assert chip select
//}
//
//// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
//void sendGraphicsLcdData(uint8_t data)
//{
//	CS_NOT = 0;                        // assert chip select
//	__asm (" NOP");                    // allow line to settle
//  	__asm (" NOP");
//    __asm (" NOP");
//	__asm (" NOP");
//	A0 = 1;                            // set A0 for data
//	SSI2_DR_R = data;                  // write data
//	while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
//	CS_NOT = 1;                        // de-assert chip select
//}
//
//void setGraphicsLcdPage(uint8_t page)
//{
//  sendGraphicsLcdCommand(0xB0 | page);
//}
//
//void setGraphicsLcdColumn(uint8_t x)
//{
//  sendGraphicsLcdCommand(0x10 | ((x >> 4) & 0x0F));
//  sendGraphicsLcdCommand(0x00 | (x & 0x0F));
//}
//
//void refreshGraphicsLcd()
//{
//    uint8_t x, page;
//    uint16_t i = 0;
//    for (page = 0; page < 8; page ++)
//    {
//    	setGraphicsLcdPage(page);
//        setGraphicsLcdColumn(0);
//        for (x = 0; x < 128; x++)
//    	    sendGraphicsLcdData(pixelMap[i++]);
//    }
//}
//
//void clearGraphicsLcd()
//{
//    uint16_t i;
//    // clear data memory pixel map
//    for (i = 0; i < 1024; i++)
//        pixelMap[i] = 0;
//    // copy to display
//    refreshGraphicsLcd();
//}
//
//void initGraphicsLcd()
//{
//    sendGraphicsLcdCommand(0x40); // set start line to 0
//    sendGraphicsLcdCommand(0xA1); // reverse horizontal order
//    sendGraphicsLcdCommand(0xC0); // normal vertical order
//    sendGraphicsLcdCommand(0xA6); // normal pixel polarity
//    sendGraphicsLcdCommand(0xA3); // set led bias to 1/9 (should be A2)
//    sendGraphicsLcdCommand(0x2F); // turn on voltage booster and regulator
//    sendGraphicsLcdCommand(0xF8); // set internal volt booster to 4x Vdd
//    sendGraphicsLcdCommand(0x00);
//    sendGraphicsLcdCommand(0x27); // set contrast
//    sendGraphicsLcdCommand(0x81); // set LCD drive voltage
//    sendGraphicsLcdCommand(0x04);
//    sendGraphicsLcdCommand(0xAC); // no flashing indicator
//    sendGraphicsLcdCommand(0x00);
//    clearGraphicsLcd();           // clear display
//    sendGraphicsLcdCommand(0xAF); // display on
//}
