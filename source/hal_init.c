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

#include <stdint.h>
#include <iso646.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include "sys_time.h"
#include "hal_init.h"
#include "sys_interrupts.h"
#include "hal_lcd.h"
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// System reset interrupt routine
void hal_ResetISR(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}
// Initialize Hardware
void hal_Init_Hardware(void)
{
//-------------------------------- SYSCTL-------------------------------------
// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN |
    		       SYSCTL_RCC_USESYSDIV  | (4 << SYSCTL_RCC_SYSDIV_S) |
				   SYSCTL_RCC_USEPWMDIV  | SYSCTL_RCC_PWMDIV_16;
// Set GPIO ports to use APB (default)
    SYSCTL_GPIOHBCTL_R = 0;
// Enable GPIO peripherals ports
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB |
    		         SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD |
					 SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
// Enable system clock to UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0
    		            |SYSCTL_RCGCUART_R1;
// Enable system clock to PWM modules
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0 | SYSCTL_RCGCPWM_R1;
// Enable system clock to ADC modules
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
// Enable system clock to SSI module
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
//-------------------------------- PORT A-------------------------------------
// Set the direction for UART0 RX and Tx
    GPIO_PORTA_DIR_R |= HAL_GPIO_BIT4 | HAL_GPIO_BIT1;
    GPIO_PORTA_DIR_R &= ~HAL_GPIO_BIT0;
// Enable UART0 pins on PORT A
    GPIO_PORTA_DEN_R |= HAL_GPIO_BIT4 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0;
// Alternate function selection for UART 0
	GPIO_PORTA_AFSEL_R |= HAL_GPIO_BIT1 | HAL_GPIO_BIT0;
// Port Control selection for UART pins
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
//-------------------------------- PORT B-------------------------------------
// Set the direction for UART1 RX and Tx
    GPIO_PORTB_DIR_R |= (HAL_GPIO_BIT7 | HAL_GPIO_BIT4 | HAL_GPIO_BIT3 | HAL_GPIO_BIT1);
    GPIO_PORTB_DIR_R &= ~(HAL_GPIO_BIT6 | HAL_GPIO_BIT5 | HAL_GPIO_BIT0);
// set drive strength to 2mA
    GPIO_PORTB_DR2R_R |= (HAL_GPIO_BIT7 | HAL_GPIO_BIT4 | HAL_GPIO_BIT3);
// Enable UART1 and SPI2 pins on PORT B
    GPIO_PORTB_DEN_R |= (HAL_GPIO_BIT7 | HAL_GPIO_BIT4 | HAL_GPIO_BIT3 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
// turn off digital operation on pins PB5
    GPIO_PORTB_DEN_R &= ~HAL_GPIO_BIT5;
// Alternate function selection for UART 1
    GPIO_PORTB_AFSEL_R |= (HAL_GPIO_BIT7 | HAL_GPIO_BIT5 | HAL_GPIO_BIT4 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
// turn on analog operation on pins PB5 for AIN11
    GPIO_PORTB_AMSEL_R |= HAL_GPIO_BIT5;
// Port Control selection for UART and SPI pins
    GPIO_PORTB_PCTL_R = (GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);
//-------------------------------- PORT C-------------------------------------
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
//-------------------------------- PORT D-------------------------------------
// Set the direction for Booster pack switches and ADC Input pins
    GPIO_PORTD_DIR_R = HAL_PORT_RESET;
// unlocking PORTD for enabling functionality of all pins
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
// enabling the functionality of all pins in PORTD
    GPIO_PORTD_CR_R = HAL_PORT_ENABLE;
// selecting alternate function on PortD pins to be ADC Input
    GPIO_PORTD_AFSEL_R |= (HAL_GPIO_BIT3 | HAL_GPIO_BIT2 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
// Enable push button pins on PORT D
    GPIO_PORTD_DEN_R |= (HAL_GPIO_BIT6 | HAL_GPIO_BIT7);
// turn off digital operation on pins PE0, PE1, PE2
    GPIO_PORTD_DEN_R &= ~(HAL_GPIO_BIT3 | HAL_GPIO_BIT2 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
// turn on analog operation on pins PE0, PE1, PE2
    GPIO_PORTD_AMSEL_R |= (HAL_GPIO_BIT3 | HAL_GPIO_BIT2 | HAL_GPIO_BIT1 | HAL_GPIO_BIT0);
// enable internal pull-up for push button
    GPIO_PORTD_PUR_R = (HAL_GPIO_BIT6 | HAL_GPIO_BIT7);
//-------------------------------- PORT E-------------------------------------
// Configure Joystick select pin PE4
    GPIO_PORTE_DIR_R &= ~HAL_GPIO_BIT4;
// make bit 1 an outputs
    GPIO_PORTE_DR2R_R |= HAL_GPIO_BIT4;
// enable internal pull-up for push button
    GPIO_PORTE_PUR_R = HAL_GPIO_BIT4;
// enable PortE
    GPIO_PORTE_DEN_R |= HAL_PORT_ENABLE;
//-------------------------------- PORT F-------------------------------------
// Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= HAL_GPIO_BIT3 | HAL_GPIO_BIT2 | HAL_GPIO_BIT1;
// make bit 1 an outputs
    GPIO_PORTF_DR2R_R |= 0x1F; //Default
// enable internal pull-up for push button
    GPIO_PORTF_PUR_R = HAL_GPIO_BIT4 | HAL_GPIO_BIT0;
// selecting alternate function on PF2 to be PWM
    GPIO_PORTF_AFSEL_R |= HAL_GPIO_BIT2;
// selecting analog mode on PF2
    GPIO_PORTF_AMSEL_R |= HAL_GPIO_BIT2;
// configure PF2 to be PWM output pin
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;
// enable LED
    GPIO_PORTF_DEN_R |= HAL_PORT_ENABLE;
// resetting the port
    GPIO_PORTF_DATA_R = HAL_PORT_RESET;
//-------------------------------- UART---------------------------------------
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
//-------------------------------- I2C----------------------------------------
//-------------------------------- PWM----------------------------------------
// disabling PWM module before configuration as buzzer on BPAC
    PWM1_3_CTL_R &= ~PWM_3_CTL_ENABLE;
// configuring pulse generator A
    PWM1_3_GENA_R |= PWM_3_GENA_ACTZERO_NONE | PWM_3_GENA_ACTLOAD_ONE |
                     PWM_3_GENA_ACTCMPAU_NONE | PWM_3_GENA_ACTCMPAD_ZERO |
                     PWM_3_GENA_ACTCMPBU_NONE | PWM_3_GENA_ACTCMPBD_NONE;
// writing the load value
    PWM1_3_LOAD_R = HAL_PWM_BUZZ_LOAD;
// loading the comparator value
    PWM1_3_CMPA_R = HAL_PWM_BUZZ_CMP;
// enabling the PWM generator
    PWM1_3_CTL_R = PWM_3_CTL_DEBUG;
// enabling the PWM module
    PWM1_3_CTL_R |= PWM_3_CTL_ENABLE;
    //PWM1_ENABLE_R |= PWM_ENABLE_PWM6EN;

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
    //PWM0_ENABLE_R |= PWM_ENABLE_PWM6EN;
//-------------------------------- SPI ---------------------------------------
// Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
// turn off SSI2 to allow re-configuration
    SSI2_CR1_R &= ~SSI_CR1_SSE;
// select master mode
    SSI2_CR1_R = 0;
// select system clock as the clock source
    SSI2_CC_R = 0;
// set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CPSR_R = 40;
// set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;
// turn on SSI2
    SSI2_CR1_R |= SSI_CR1_SSE;
    HAL_LCD_CS = 1;
//------------------------------- TIMERS--------------------------------------
//-------------------------------- ADC----------------------------------------
// select PLL as the time base (not needed, since default value)
	ADC0_CC_R = ADC_CC_CS_SYSPLL;
// disable sample sequencer 3 (SS3) for programming
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;
// select SS3 bit in ADCPSSI as trigger
	ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;
// set first sample to analog input pin
	ADC0_SSMUX3_R = HAL_BPAC_ACC_AXIS_X_AIN;
// mark first sample as the end
	ADC0_SSCTL3_R = ADC_SSCTL3_END0;
// enable SS3 for operation
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
//------------------------------- EEPROM--------------------------------------
}

void hal_Application_Start()
{
	uint16_t raw_input;
	uint8_t acc_xaxis_output;
	double scaled_value;
	while(1)
	{
		raw_input = hal_ADC0_readSs3();
		scaled_value = ((((double)raw_input-1000.0)*256.0)/2000.0);
		acc_xaxis_output = (uint8_t)floor(scaled_value);
		//while (UART0_FR_R & UART_FR_TXFF);
		//UART0_DR_R = (uint8_t)acc_xaxis_output;
		while (UART1_FR_R & UART_FR_TXFF);
		UART1_DR_R = (uint8_t)acc_xaxis_output;
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) = HAL_LED_ON;
		//hal_SW4_press_wait();
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) = HAL_LED_OFF;
		//HAL_BPAC_BLU_LED_ON;
		//HAL_BPAC_BUZZ_ON;
		//hal_PWM_BUZZ_control(true);

		//hal_SW4_release_wait();
		//UART0_DR_R = 0x33;
		//HAL_BPAC_BLU_LED_OFF;
		//HAL_BPAC_BUZZ_OFF;
		//hal_PWM_BUZZ_control(false);
		//hal_LCD_Senddata(0x46);
//		while(HAL_BPAC_SWJ_STATUS);
//		HAL_LPAD_RED_LED_ON;
//		while(!HAL_BPAC_SWJ_STATUS);
//		HAL_LPAD_RED_LED_OFF;
		sys_delay_usec(DELAY_1SEC/10);
		//HAL_LED_SELECT(HAL_LED_PIN_GREEN) ^= HAL_LED_TOGGLE;
	}
}
int16_t hal_ADC0_readSs3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

// Blocking function that returns only when SW1 is pressed
void hal_SW1_press_wait(void)
{
	while(HAL_LPAD_SW1_STATUS);
}

// Blocking function that returns only when SW1 is released
void hal_SW1_release_wait(void)
{
	while(!HAL_LPAD_SW1_STATUS);
}

// Blocking function that returns only when SW2 is pressed
void hal_SW2_press_wait(void)
{
	while(!HAL_LPAD_SW2_STATUS);
}

// Blocking function that returns only when SW2 is released
void hal_SW2_release_wait(void)
{
	while(HAL_LPAD_SW2_STATUS);
}

// Blocking function that returns only when SW3 is pressed
void hal_SW3_press_wait(void)
{
	while(HAL_BPAC_SW1_STATUS);
}

// Blocking function that returns only when SW3 is released
void hal_SW3_release_wait(void)
{
	while(!HAL_BPAC_SW1_STATUS);
}

// Blocking function that returns only when SW4 is pressed
void hal_SW4_press_wait(void)
{
	while(HAL_BPAC_SW2_STATUS);
}

// Blocking function that returns only when SW4 is released
void hal_SW4_release_wait(void)
{
	while(!HAL_BPAC_SW2_STATUS);
}
