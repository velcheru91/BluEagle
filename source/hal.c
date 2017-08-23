/*
 * hal_init.c
 *
 *  Created on: Aug 25, 2016
 *      Author: Venugopal Velcheru
 */
//-----------------------------------------------------------------------------
// Hardware Abstraction Layer Initialization
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#ifndef _STDINT_H_
#include <stdint.h>
#endif
#ifndef _STDBOOL
#include <stdbool.h>
#endif
#ifndef _MATH
#include <math.h>
#endif
#ifndef __TM4C123GH6PM_H__
#include <tm4c123gh6pm.h>
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
// System reset interrupt routine
void HAL_Reset_ISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void HAL_Sys_Delay(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
// Initialization routines which are called only once
// Initialize the system clock
void HAL_Sys_Clk_Init(void)
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN     |
	    		   SYSCTL_RCC_USESYSDIV  | (4 << SYSCTL_RCC_SYSDIV_S) |
				   SYSCTL_RCC_USEPWMDIV  | SYSCTL_RCC_PWMDIV_16;
	// Set GPIO ports to use APB (default)
	// Enable GPIO peripherals ports
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB |
	    		     SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD |
				     SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
	SYSCTL_GPIOHBCTL_R = 0;
}

void HAL_Button1_Init(void)
{
	// Set the direction for Booster pack switches and ADC Input pins
	GPIO_PORTD_DIR_R = HAL_PORT_RESET;
	// unlocking PORTD for enabling functionality of all pins
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
	// Enable push button pin on PORTD PIN6
	GPIO_PORTD_DEN_R |= (HAL_GPIO_BIT6);
	// enable internal pull-up for push button
	GPIO_PORTD_PUR_R |= (HAL_GPIO_BIT6);
}

void HAL_Button2_Init(void)
{
	// Set the direction for Booster pack switches and ADC Input pins
	GPIO_PORTD_DIR_R = HAL_PORT_RESET;
	// unlocking PORTD for enabling functionality of all pins
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
	// Enable push button pin on PORTD PIN6
	GPIO_PORTD_DEN_R |= (HAL_GPIO_BIT7);
	// enable internal pull-up for push button
	GPIO_PORTD_PUR_R |= (HAL_GPIO_BIT7);
}

void HAL_Accelero_Init(void)
{

}

void HAL_RGB_BPACK_Init(void)
{
	// Initializing PB3 for Green LED
	// Enable system clock to PWM modules
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
	// Set the direction for LED control
	GPIO_PORTB_DIR_R |= HAL_GPIO_BIT3;
	// Enable pin 3 on PORT B
	GPIO_PORTB_DEN_R |= HAL_GPIO_BIT3;

	// Initializing PC4 for Blue LED
	// Configure LED pin
	GPIO_PORTC_DIR_R |= HAL_GPIO_BIT4;
	// make bit 1 an outputs
	GPIO_PORTC_DR2R_R |= HAL_GPIO_BIT4;
	// selecting alternate function on PC4d to be PWM
	GPIO_PORTC_AFSEL_R |= HAL_GPIO_BIT4;
	// selecting analog mode on PC4
	GPIO_PORTC_AMSEL_R |= HAL_GPIO_BIT4;
	// configure PC4 to be PWM output pin
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6;
	// enable LED
	GPIO_PORTC_DEN_R |= HAL_GPIO_BIT4;
	// resetting the port
	GPIO_PORTC_DATA_R = HAL_PORT_RESET;
	// disabling PWM module before configuration as BLUE LED on BPAC
	PWM0_3_CTL_R &= ~PWM_3_CTL_ENABLE;
	// configuring pulse generator A
	PWM0_3_GENA_R |= PWM_3_GENA_ACTZERO_NONE | PWM_3_GENA_ACTLOAD_ONE |
	                 PWM_3_GENA_ACTCMPAU_NONE | PWM_3_GENA_ACTCMPAD_ZERO |
	                 PWM_3_GENA_ACTCMPBU_NONE | PWM_3_GENA_ACTCMPBD_NONE;
	// writing the load value
	PWM0_3_LOAD_R = HAL_PWM_BUZZ_LOAD;
	// loading the comparator value
	PWM0_3_CMPA_R = HAL_PWM_BUZZ_CMP;
	// enabling the PWM generator
	PWM0_3_CTL_R = PWM_3_CTL_DEBUG;
	// enabling the PWM module
	PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;
}

void HAL_Buzzer_Init(void)
{
	// Enable system clock to PWM modules
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
	// Configure pins for Buzzer
	GPIO_PORTF_DIR_R |= HAL_GPIO_BIT2;
	// selecting alternate function on PF2 to be PWM
	GPIO_PORTF_AFSEL_R |= HAL_GPIO_BIT2;
	// selecting analog mode on PF2
	GPIO_PORTF_AMSEL_R |= HAL_GPIO_BIT2;
	// configure PF2 to be PWM output pin
	GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;
	// enable LED
	GPIO_PORTF_DEN_R |= HAL_GPIO_BIT2;
	// resetting the port
	//GPIO_PORTF_DATA_R = HAL_PORT_RESET;
	// disabling PWM module before configuration as buzzer on BPAC
	PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
	// configuring pulse generator A
	PWM1_3_GENA_R |= PWM_3_GENA_ACTZERO_NONE | PWM_3_GENA_ACTLOAD_ONE |
	                 PWM_3_GENA_ACTCMPAU_NONE | PWM_3_GENA_ACTCMPAD_ZERO |
	                 PWM_3_GENA_ACTCMPBU_NONE | PWM_3_GENA_ACTCMPBD_NONE;
	// writing the load value
	PWM1_3_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
//	PWM1_3_LOAD_R = 0xC3500;//HAL_PWM_BUZZ_LOAD;
	// loading the comparator value
	PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
	// enabling the PWM generator
	PWM1_3_CTL_R = PWM_3_CTL_DEBUG;
	// enabling the PWM module
	PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
}

void HAL_Servo1_Init(void)
{
    // Enable system clock to PWM modules
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    // Configure pins for Buzzer
    GPIO_PORTE_DIR_R |= HAL_GPIO_BIT5;
    // make bit 1 an outputs
    GPIO_PORTE_DR2R_R |= HAL_GPIO_BIT5;
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTE_AFSEL_R |= HAL_GPIO_BIT5;
    // selecting analog mode on PF2
    GPIO_PORTE_AMSEL_R |= HAL_GPIO_BIT5;
    // configure PF2 to be PWM output pin
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE5_M1PWM3;
    // enable LED
    GPIO_PORTE_DEN_R |= HAL_GPIO_BIT5;
    // resetting the port
    GPIO_PORTE_DATA_R = HAL_PORT_RESET;
    // disabling PWM module before configuration as buzzer on BPAC
    PWM1_1_CTL_R &= ~PWM_2_CTL_ENABLE;
    // configuring pulse generator A
    PWM1_1_GENA_R |= PWM_2_GENA_ACTZERO_NONE | PWM_2_GENA_ACTLOAD_ONE |
                     PWM_2_GENA_ACTCMPAU_NONE | PWM_2_GENA_ACTCMPAD_ZERO |
                     PWM_2_GENA_ACTCMPBU_NONE | PWM_2_GENA_ACTCMPBD_NONE;
    // writing the load value
    PWM1_1_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
    // loading the comparator value
    PWM1_1_CMPA_R = HAL_PWM_BUZZ_CMP;
    // enabling the PWM generator
    PWM1_1_CTL_R = PWM_1_CTL_DEBUG;
    // enabling the PWM module
    PWM1_1_CTL_R |= PWM_1_CTL_ENABLE;
}

void HAL_Servo2_Init(void)
{
    // Enable system clock to PWM modules
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    // Configure pins for Buzzer
    GPIO_PORTC_DIR_R |= HAL_GPIO_BIT5;
    // make bit 1 an outputs
    GPIO_PORTC_DR2R_R |= HAL_GPIO_BIT5;
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTC_AFSEL_R |= HAL_GPIO_BIT5;
    // selecting analog mode on PF2
    GPIO_PORTC_AMSEL_R |= HAL_GPIO_BIT5;
    // configure PF2 to be PWM output pin
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_M0PWM7;
    // enable LED
    GPIO_PORTC_DEN_R |= HAL_GPIO_BIT5;
    // resetting the port
    GPIO_PORTC_DATA_R = HAL_PORT_RESET;
    // disabling PWM module before configuration as buzzer on BPAC
    PWM0_3_CTL_R &= ~PWM_3_CTL_ENABLE;
    // configuring pulse generator A
    PWM0_3_GENA_R |= PWM_2_GENA_ACTZERO_NONE | PWM_2_GENA_ACTLOAD_ONE |
                     PWM_2_GENA_ACTCMPAU_NONE | PWM_2_GENA_ACTCMPAD_ZERO |
                     PWM_2_GENA_ACTCMPBU_NONE | PWM_2_GENA_ACTCMPBD_NONE;
    // writing the load value
    PWM0_3_LOAD_R = HAL_PWM_BUZZ_LOAD;//0xC34F;//0xC3500;//
    // loading the comparator value
    PWM0_3_CMPA_R = HAL_PWM_BUZZ_CMP;
    // enabling the PWM generator
    PWM0_3_CTL_R = PWM_0_CTL_DEBUG;
    // enabling the PWM module
    PWM0_3_CTL_R |= PWM_0_CTL_ENABLE;
}

void static adcinit(void)
{
	// Enable system clock to ADC modules
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
	// Allow time for clock to stabilize
	while((SYSCTL_PRADC_R&0x01) == 0){};
    // GPIO initialization in more specific functions
	// select PLL as the time base (not needed, since default value)
	ADC0_CC_R = ADC_CC_CS_SYSPLL;
	//clear max sample rate field
	ADC0_PC_R &= ~ADC_SPC_PHASE_M;
	//configure for 125K samples/sec
	ADC0_PC_R |= ADC_SPC_PHASE_22_5;
    // Sequencer 3 is lowest priority
	ADC0_SSPRI_R = 0x3210;
}

void HAL_Microphone_Init()
{

}

void HAL_Joystick_Init(void)
{
	// Set the direction for UART1 RX and Tx
	GPIO_PORTB_DIR_R &= ~(HAL_GPIO_BIT5);
	// turn off digital operation on pins PB5
	GPIO_PORTB_DEN_R &= ~HAL_GPIO_BIT5;
	// Alternate function selection for Joystick x
	GPIO_PORTB_AFSEL_R |= HAL_GPIO_BIT5;
	// turn on analog operation on pins PB5 for AIN11
	GPIO_PORTB_AMSEL_R |= HAL_GPIO_BIT5;

	// Set the direction for Booster pack switches and ADC Input pins
	GPIO_PORTD_DIR_R = HAL_PORT_RESET;
	// unlocking PORTD for enabling functionality of all pins
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
	// selecting alternate function on PortD pins to be ADC Input
	GPIO_PORTD_AFSEL_R |= HAL_GPIO_BIT3;
	// turn off digital operation on pins PD3
	GPIO_PORTD_DEN_R &= ~HAL_GPIO_BIT3;
	// turn on analog operation on pins PD3
	GPIO_PORTD_AMSEL_R |= HAL_GPIO_BIT3;

	// unlocking PORTE for enabling functionality of all pins
	GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;
	// enabling the functionality of all pins in PORTD
	GPIO_PORTE_CR_R = HAL_PORT_ENABLE;
	// Configure Joystick select pin PE4
	GPIO_PORTE_DIR_R &= ~HAL_GPIO_BIT4;
	// make bit 1 an outputs
	GPIO_PORTE_DR2R_R |= HAL_GPIO_BIT4;
	// enable internal pull-up for push button
	GPIO_PORTE_PUR_R |= HAL_GPIO_BIT4;
	// enable PortE
	GPIO_PORTE_DEN_R |= HAL_GPIO_BIT4;
	adcinit();

	// disable sample sequencer 1 (SS1) for programming
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;
	// select SS1 bit in ADCPSSI as trigger
	ADC0_EMUX_R &= ~ADC_EMUX_EM1_M;
	// set first and second samples to AIN4 and AIN11
	ADC0_SSMUX1_R = 0x004B;
	// no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
	ADC0_SSCTL1_R = ADC_SSCTL1_IE1 | ADC_SSCTL1_END1;
	// disable SS1 interrupts
	ADC0_IM_R &= ~ADC_IM_MASK1;
	// enable sample sequencer 1
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;
}

void HAL_RGB_LPAD_Init(void)
{
	// With BooserPack attached we can only use RED LED on LaunchPad
	// Configure LED and pushbutton pins
	GPIO_PORTF_DIR_R |= HAL_GPIO_BIT1;
	// enable LED
	GPIO_PORTF_DEN_R |= HAL_GPIO_BIT1;
	// resetting the pin
	GPIO_PORTF_DATA_R &= !HAL_GPIO_BIT1;
}

void HAL_ButtonLPAD_Init(void)
{

}

void HAL_BT_UART_Init(void)
{
	//Enable system clock to UART1
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
	// Set the direction for UART1 RX and Tx
	GPIO_PORTB_DIR_R |= (HAL_GPIO_BIT1);
	GPIO_PORTB_DIR_R &= ~(HAL_GPIO_BIT0);
	// Enable UART1 and SPI2 pins on PORT B
	GPIO_PORTB_DEN_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
	// Alternate function selection for UART 1
	GPIO_PORTB_AFSEL_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
	// Port Control selection for UART and SPI pins
	GPIO_PORTB_PCTL_R = (GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);

	// turn-off UART1 to allow safe programming
	UART1_CTL_R = 0;
	// use system clock (40 MHz)
	UART1_CC_R = UART_CC_CS_SYSCLK;
	// r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART1_IBRD_R = 260;//21;
	// round(fract(r)*64)=45
	UART1_FBRD_R = 27;//45;
	// configure for 8N1 w/ 16-level FIFO disabled
	UART1_LCRH_R = UART_LCRH_WLEN_8;
	// enable TX, RX, and module
	UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
	// enabling interrupt on UART1 RX
	UART1_IM_R = UART_IM_RXIM;
	// setting UART1 bit in vector table
	NVIC_EN0_R = 1<<6;
}

void HAL_LPAD_UART_Init(void)
{
	//Enable system clock to UART0
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
	// Set the direction for UART0 RX and Tx
	GPIO_PORTA_DIR_R |= HAL_GPIO_BIT1;
	GPIO_PORTA_DIR_R &= ~HAL_GPIO_BIT0;
	// Enable UART0 pins on PORT A
	GPIO_PORTA_DEN_R |= (HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
	// Alternate function selection for UART 0
	GPIO_PORTA_AFSEL_R |= HAL_GPIO_BIT1 | HAL_GPIO_BIT0;
	// Port Control selection for UART pins
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// turn-off UART0 to allow safe programming
	UART0_CTL_R = 0;
	// use system clock (40 MHz)
	UART0_CC_R = UART_CC_CS_SYSCLK;
	// r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_IBRD_R = 260;//21;
	// round(fract(r)*64)=45
	UART0_FBRD_R = 27;//45;
	// configure for 8N1 w/ 16-level FIFO disabled
	UART0_LCRH_R = UART_LCRH_WLEN_8;
	// enable TX, RX, and module
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
	// enabling interrupt on UART0 RX
	UART0_IM_R = UART_IM_RXIM;
	// setting UART1 bit in vector table
	NVIC_EN0_R = 1<<5;
}
/*----------------------------------------------------------------*/
// Usage routines which are called as per task rate
bool HAL_Button1_Input(void)
{
	return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT6)>>6)?false:true);
}

bool HAL_Button2_Input(void)
{
	return(((GPIO_PORTD_DATA_R & HAL_GPIO_BIT7)>>7)?false:true);
}

void HAL_Accelero_Input(void)
{

}
void HAL_RGB_BPACK_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	red++; // to avoid compiler warning
	(green?(HAL_BPAC_GRN_LED_ON):(HAL_BPAC_GRN_LED_OFF));
	(blue?(HAL_BPAC_BLU_LED_ON):(HAL_BPAC_BLU_LED_OFF));
}
void HAL_Buzzer_Set(uint8_t buzz, uint8_t tone)
{
	switch(tone)
	{
	case 0://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		   PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.925);//0xEA4;//0xB4AA0;
//		   PWM1_3_CMPA_R = 0xB4AA0;
//		   PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	case 1:PWM1_3_CMPA_R = (2500*0.20)-1;
		break;
	case 2:PWM1_3_CMPA_R = (2500*0.30)-1;
		break;
	case 3:PWM1_3_CMPA_R = (2500*0.40)-1;
		break;
	case 4:PWM1_3_CMPA_R = (2500*0.50)-1;
		break;
	case 5:PWM1_3_CMPA_R = (2500*0.60)-1;
		break;
	case 6:PWM1_3_CMPA_R = (2500*0.70)-1;
		break;
	case 7:PWM1_3_CMPA_R = (2500*0.80)-1;
		break;
	case 90://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		     PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);//0x1387;
//			 PWM1_3_CMPA_R = 0xAFCE4;
//		     PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	case 180://PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
		      PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.95);//0x9C3;
//			  PWM1_3_CMPA_R = 0xB985C;
//		      PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
		break;
	default:PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
		break;
	}
	(buzz?(HAL_BPAC_BUZZ_ON):(HAL_BPAC_BUZZ_OFF));
}

void HAL_Microphone_Input(void)
{

}
void HAL_Joystick_Input(uint16_t* x, uint16_t* y, uint32_t* select)
{
	// initiate SS1
	ADC0_PSSI_R |= ADC_PSSI_SS1;
	// wait for conversion done
	while((ADC0_RIS_R & ADC_RIS_INR1)==0){};
	while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);
	// read first result
	*x = ADC0_SSFIFO1_R>>2;
	// read second result
	*y = ADC0_SSFIFO1_R>>2;
	// return 0(pressed) or 0x10(not pressed)
	*select = HAL_BPAC_SWJ_STATUS;
	// acknowledge completion
	ADC0_ISC_R = ADC_ISC_IN1;
}
void HAL_RGB_LPAD_Set(uint8_t red, uint8_t green, uint8_t blue)
{
	(red?(HAL_LPAD_RED_LED_ON):(HAL_LPAD_RED_LED_OFF));
	blue++; // to avoid compiler warning
	green++; // to avoid compiler warning
}

void HAL_LPAD_UART_Write(uint8_t data)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = data;

}
void HAL_LPAD_UART_Read(uint8_t* data)
{
	while (UART0_FR_R & UART_FR_RXFE);
	*data = UART0_DR_R;
}
//int16_t hal_ADC0_readSs3()
//{
//    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
//               // wait until SS3 is not busy
//    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
//}

void HAL_Switch_Init(void)
{
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    // enabling the functionality of all pins in PORTD
    GPIO_PORTF_CR_R = HAL_PORT_ENABLE;
    // enable LED
    GPIO_PORTF_DIR_R &= ~(HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
    // selecting alternate function on PF2 to be PWM
    GPIO_PORTF_PUR_R |= (HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
//    GPIO_PORTF_AFSEL_R |= HAL_GPIO_BIT2;
    GPIO_PORTF_DEN_R |= (HAL_GPIO_BIT0 | HAL_GPIO_BIT4);
    // selecting analog mode on PF2
//    GPIO_PORTF_AMSEL_R |= HAL_GPIO_BIT2;
    // configure PF2 to be PWM output pin
//    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;

}

void HAL_Init(void)
{
	HAL_Sys_Clk_Init();
//	HAL_RGB_LPAD_Init();
//	HAL_RGB_LPAD_Set(1,0,0);
//	HAL_Button1_Init();
//	HAL_Button2_Init();
	HAL_Switch_Init();
	HAL_RGB_BPACK_Init();
//	HAL_Sys_Delay(DELAY_1SEC);
	HAL_Buzzer_Init();
//	HAL_Servo1_Init();
//	HAL_Servo2_Init();
//	HAL_Joystick_Init();
//	HAL_LPAD_UART_Init();
	//HAL_LCD_Init();
	//HAL_Sys_Delay(.5*DELAY_1SEC);
//	HAL_RGB_LPAD_Set(0,0,0);
}

void HAL_Application_Start()
{
//	uint32_t* sel_pt;
//	uint16_t raw_x, raw_y;
//	uint8_t out_x, out_y;
    uint8_t dir1=1,dir2=1;
    uint32_t j=850,k=880, data1=0, data2=0;
//	double scale_value_x, scale_value_y;
	PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90); // reset to position 0
	PWM1_ENABLE_R |= PWM_ENABLE_PWM6EN;

//	PWM1_1_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
//	PWM1_ENABLE_R |= PWM_ENABLE_PWM3EN;

    PWM0_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
    PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN;

//    PWM0_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.90);
//    PWM0_ENABLE_R |= PWM_ENABLE_PWM7EN;
	while(1)
	{
        if ((GPIO_PORTF_DATA_R & 0x01) == 0)
        {
            if (dir1 == 1)
                j++;
            else
                j--;
            if (j>=915)
                dir1 = 0;
            if (j<=850)
                dir1 = 1;

        //for(j=850;j<=915;j=j+1)
        //{
            data1 = (((HAL_PWM_BUZZ_LOAD-1)*j)/1000);
            PWM1_3_CMPA_R = data1;
//          PWM0_3_CMPA_R = data;
            HAL_Sys_Delay(20000);
        //}
//        for(j=915;j>=850;j=j-1)
//        {
//            data1 = (((HAL_PWM_BUZZ_LOAD-1)*j)/1000);
//            PWM1_3_CMPA_R = data1;
//          PWM0_3_CMPA_R = data2;
//            HAL_Sys_Delay(20000);
//        }
        }

        if ((GPIO_PORTF_DATA_R & 0x10) == 0)
        {
            if (dir2 == 1)
                k++;
            else
                k--;
            if (k>=940)
                dir2 = 0;
            if (k<=880)
                dir2 = 1;
//        for(k=940;k>=880;k=k-1)
//        {
            data2 = (((HAL_PWM_BUZZ_LOAD-1)*k)/1000);
//            PWM1_3_CMPA_R = data1;
            PWM0_3_CMPA_R = data2;
            HAL_Sys_Delay(20000);
//        }
//        for(k=880;k<=940;k=k+1)
//        {
//            data2 = (((HAL_PWM_BUZZ_LOAD-1)*k)/1000);
//            PWM1_3_CMPA_R = data;
//            PWM0_3_CMPA_R = data2;
//            HAL_Sys_Delay(20000);
//        }
        }
//		HAL_Joystick_Input(&raw_x, &raw_y, sel_pt);
//		scale_value_x = ((((double)(raw_x)-1000.0)*256.0)/4096.0);
//		out_x = (uint8_t)floor(scale_value_x);
//		scale_value_y = ((((double)(raw_y)-1000.0)*256.0)/4096.0);
//		out_y = (uint8_t)floor(scale_value_y);
		//HAL_LPAD_UART_Write(out_x);
		//HAL_LPAD_UART_Write(out_y);
		//HAL_LPAD_UART_Write(0x0A);
//		if(HAL_Button1_Input())
//		{
//			HAL_RGB_BPACK_Set(0,1,0);
//			HAL_Buzzer_Set(1,0);
//		}
//		else if(HAL_Button2_Input())
//		{
//			HAL_RGB_BPACK_Set(0,0,1);
//			HAL_Buzzer_Set(1,1);
//		}
//		else if(!(((*sel_pt) & HAL_GPIO_BIT4)>>4))
//		{
//			HAL_RGB_BPACK_Set(0,0,0);
//			HAL_Buzzer_Set(1,2);
//		}
//		else
//		{
//			HAL_Buzzer_Set(0,0);
//		}
//		uint8_t i,j;
//		for (i=0;i<8;i++)
//		{
//			HAL_Buzzer_Set(0,0);
//			HAL_Buzzer_Set(1,i);
//			HAL_Sys_Delay(DELAY_1mSEC*500);
//		}
//		for (j=8;j>0;j--)
//		{
//			HAL_Buzzer_Set(0,0);
//			HAL_Buzzer_Set(1,j-1);
		//if(HAL_Button1_Input())

// 		HAL_Sys_Delay(DELAY_1SEC);
// 		{
// 			PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.950);
//			HAL_BPAC_BUZZ_ON
// 		}
// 		HAL_Sys_Delay(DELAY_1SEC);
		//else if (HAL_Button2_Input())
// 		{
// 			PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.925);
// 		}
// 		HAL_Sys_Delay(DELAY_1SEC);
		//else
// 		{
// 			PWM1_3_CMPA_R = ((HAL_PWM_BUZZ_LOAD-1)*0.900);
// 		}

//		HAL_Sys_Delay(DELAY_1SEC);
//
//		HAL_Sys_Delay(DELAY_1SEC);
//
//		HAL_Sys_Delay(DELAY_1SEC);
//		HAL_Buzzer_Set(1,0);
//		HAL_Sys_Delay(DELAY_1SEC);
//		}
	}
		//raw_input = hal_ADC0_readSs3();
		//scaled_value = ((((double)raw_input-1000.0)*256.0)/2000.0);
		//acc_xaxis_output = (uint8_t)floor(scaled_value);
		//while (UART0_FR_R & UART_FR_TXFF);
		//UART0_DR_R = (uint8_t)acc_xaxis_output;
		//while (UART1_FR_R & UART_FR_TXFF);
		//UART1_DR_R = (uint8_t)acc_xaxis_output;
}
//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

void _ISR_HAL_UART0_RX_ (void)
{
/*	uint8_t var_bridge;
	while (UART0_FR_R & UART_FR_RXFE);
	var_bridge = UART0_DR_R;
	while (UART1_FR_R & UART_FR_TXFF);
	UART1_DR_R = var_bridge;*/
}
void _ISR_HAL_UART1_RX_ (void)
{
/*	uint8_t var_bridge;
	while (UART1_FR_R & UART_FR_RXFE);
	var_bridge = UART1_DR_R;
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = var_bridge;*/
}
